using ModularAgents.Kinematic;
using ModularAgents.Kinematic.Mujoco;
using ModularAgents;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using System;
using Unity.VisualScripting;

namespace Mujoco.Extensions
{
    /// <summary>
    /// Ballistic targeting with first order approx of target position.
    /// </summary>
    public class MjProjectilePerturbation : TrainingEventHandler
    {
        [SerializeField]
        Transform origin;

        [SerializeField]
        bool matchHeight;

        [SerializeField]
        Transform target;
        IKinematic targetKinematics;

        [SerializeField]
        MjFreeJoint projectileDofs;

        [SerializeField]
        float startingVelocity;

        /// <summary>
        /// Toggle for using velocity based prediction. 
        /// </summary>
        [SerializeField]
        bool useApprox;


        bool hasInitialized;

        public override EventHandler Handler => LaunchProjectile;

        private unsafe void LaunchProjectile(object sender, EventArgs e)
        {
            if (!hasInitialized)
            {
                Initialize();
                hasInitialized = true;
            }


            var (x0, y0) = (X0, Y0);
            var (x0Dot, y0Dot) = (X0Dot, Y0Dot);
            var g = -Physics.gravity.y; // No support for non-standard flatlands. Don't use this for ICBMs!
            var alpha0 = Mathf.Atan2(y0, x0);
            var optimalTheta0 = Mathf.PI / 4 + alpha0 / 2;
            var minV = Mathf.Sqrt(Sq(x0) * g / (x0 * Mathf.Sin(2 * optimalTheta0) - 2 * Y0 * Sq(Mathf.Cos(optimalTheta0))));
            var v0 = Mathf.Max(startingVelocity, minV);

            var theta0 = Mathf.Atan2(Sq(v0) - Mathf.Sqrt(Mathf.Max(Sq(Sq(v0)) - g * (g * Sq(x0) + 2 * x0 * Sq(0)), 0)), g * X0); // will be the same as optimalTheta0 if startingVelocity <= minV

            if (!useApprox)
            {
                var xUnit0 = GetHorizontalDirection0();
                Vector3 rotAxis0 = new Vector3(-xUnit0.z, 0, xUnit0.x);

                var aimVector0 = Quaternion.AngleAxis(theta0 * Mathf.Rad2Deg, rotAxis0) * xUnit0;
                var vFinal0 = aimVector0 * v0;
                Launch(vFinal0);
                return;
            }

            var flightTime = x0 / (v0 * Mathf.Cos(theta0) - x0Dot);

            // Smoothing velocity would probably lead to better results than just using these raw values.
            var xHat = x0 + flightTime * x0Dot;
            var yHat = y0; // + flightTime * y0Dot * 0.5f; // The first order approx is much less accurate on y so lets skip it

            // Honestly, the difference between theta0 and theta will usuall be very small, but as an exercise:
            var theta = Mathf.Atan2(Sq(v0) - Mathf.Sqrt(Mathf.Max(Sq(Sq(v0)) - g * (g * Sq(xHat) + 2 * yHat * Sq(v0)), 0)), g * xHat);

            var xUnit = GetHorizontalDirection(flightTime);
            Vector3 rotAxis = new Vector3(-xUnit.z, 0, xUnit.x);

            var aimVector = Quaternion.AngleAxis(theta * Mathf.Rad2Deg, rotAxis) * xUnit;

            var vFinal = aimVector * v0;

            Launch(vFinal);


        }

        private unsafe void Launch(Vector3 velocity)
        {
            MjState.TeleportMjRoot(projectileDofs, StartPos, false);
            MjEngineTool.SetMjVector3(MjScene.Instance.Data->qvel + projectileDofs.DofAddress, velocity);
            MjEngineTool.SetMjVector3(MjScene.Instance.Data->qvel + projectileDofs.DofAddress + 3, Vector3.zero);
        }

        private void OnDrawGizmosSelected()
        {
            if (!hasInitialized) return;

            var (x0, y0) = (X0, Y0);
            var (x0Dot, y0Dot) = (X0Dot, Y0Dot);
            var g = -Physics.gravity.y; // No support for non-standard flatlands. Don't use this for ICBMs!
            var alpha0 = Mathf.Atan2(y0, x0);
            var optimalTheta0 = Mathf.PI / 4 + alpha0 / 2;
            var minV = Mathf.Sqrt(Sq(x0) * g / (x0 * Mathf.Sin(2 * optimalTheta0) - 2 * Y0 * Sq(Mathf.Cos(optimalTheta0))));
            var v0 = Mathf.Max(startingVelocity, minV);

            var theta0 = Mathf.Atan2(Sq(v0) - Mathf.Sqrt(Mathf.Max(Sq(Sq(v0)) - g * (g * Sq(x0) + 2 * x0 * Sq(0)), 0)), g * X0); // will be the same as optimalTheta0 if startingVelocity <= minV
            var flightTime = x0 / (v0 * Mathf.Cos(theta0) - x0Dot);
            var xHat = x0 + flightTime * x0Dot;
            var yHat = y0; // + flightTime * y0Dot * 0.5f; // The first order approx is much less accurate on y so lets skip it

            // Honestly, the difference between theta0 and theta will usuall be very small, but as an exercise:
            var theta = Mathf.Atan2(Sq(v0) - Mathf.Sqrt(Mathf.Max(Sq(Sq(v0)) - g * (g * Sq(xHat) + 2 * yHat * Sq(v0)), 0)), g * xHat);

            var xUnit = GetHorizontalDirection(flightTime);
            Vector3 rotAxis = new Vector3(-xUnit.z, 0, xUnit.x);

            var aimVector = Quaternion.AngleAxis(theta * Mathf.Rad2Deg, rotAxis) * xUnit;


            Gizmos.color = Color.red;
            Gizmos.DrawLine(StartPos, StartPos + rotAxis);

            Gizmos.color = Color.green;
            Gizmos.DrawLine(StartPos, StartPos + aimVector);

            Gizmos.color = Color.blue;
            Gizmos.DrawLine(StartPos, StartPos + xUnit);
        }


        private float X0 => (targetKinematics.Position - StartPos).Horizontal3D().magnitude;

        private float X0Dot => Vector3.Dot(targetKinematics.Velocity, GetHorizontalDirection0());

        private float Y0 => targetKinematics.Position.y - StartPos.y;

        private float Y0Dot => targetKinematics.Velocity.y;


        private Vector3 StartPos => new Vector3(origin.position.x, matchHeight ? target.position.y : origin.position.y, origin.position.z);



        private Vector3 GetHorizontalDirection0() => (targetKinematics.Position - origin.position).Horizontal3D().normalized;

        private Vector3 GetHorizontalDirection(float flightTime) => (targetKinematics.Position + targetKinematics.Velocity * flightTime - origin.position).Horizontal3D().normalized;



        private void Initialize()
        {
            targetKinematics = target.GetIKinematic();

        }

        private float Sq(float x) => x * x;
    }
}