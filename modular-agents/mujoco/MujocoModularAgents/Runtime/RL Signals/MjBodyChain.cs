using ModularAgents.Kinematic;
using Mujoco;
using Mujoco.Extensions;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;

namespace ModularAgents.Kinematic.Mujoco
{
    public class MjBodyChain : BodyChain
    {


        public MjBodyChain(Transform chainRoot) : base(chainRoot) { }

        public MjBodyChain(IEnumerable<Transform> bodies)
        {
            chain = bodies.Select(mjT => mjT.GetIKinematic()).ToList().AsReadOnly();

            mass = chain.Select(k => k.Mass).Sum();
        }

        protected override IReadOnlyList<IKinematic> GetKinematicChain(Transform root)
        {
            return root.GetComponentsInChildren<Transform>().Where(t => t.IsIKinematic()).Select(mjT => mjT.GetIKinematic()).ToList().AsReadOnly();
        }
    }

    public static class MjKinematicExtensions
    {
        public static BodyChain GetBodyChain(this Transform transform)
        {
            return new MjBodyChain(transform);
        }

        public static BodyChain GetBodyChain(this IEnumerable<Transform> bodies)
        {
            return new MjBodyChain(bodies);
        }

        public static IKinematic GetIKinematic(this Transform transform)
        {
            if (transform.GetComponent<MjBody>())
            {
                return new MjBodyAdapter(transform.GetComponent<MjBody>());
            }
            else if(transform.GetComponent<MjMocapBodyKinematicsComponent>())
            {
                return transform.GetComponent<MjMocapBodyKinematicsComponent>().GetIKinematic();
            }
            throw new NotImplementedException($"No kinematic component recognized on transform {transform.name}");
        }

        public static bool IsIKinematic(this Transform transform)
        {
            return transform.GetComponent<ArticulationBody>() || transform.GetComponent<Rigidbody>() || transform.GetComponent<MjBody>() || transform.GetComponent<MjMocapBodyKinematicsComponent>();
        }
    }



    public class MjBodyAdapter : IKinematic
    {
        readonly private MjBody mjBody;

        readonly private MjScene scene;

        private float mass;

        private Vector3 inertiaLocalPos;

        private Matrix4x4 inertiaRelMatrix;

        private MjBody parentBody;

        public int index { get => mjBody.MujocoId; }

        public readonly bool isRoot;

        public unsafe MjBodyAdapter(MjBody mjBody)
        {
            this.mjBody = mjBody;

            MjState.ExecuteAfterMjStart(MjInitialize);

            parentBody = mjBody.transform.parent.GetComponent<MjBody>();
            isRoot = parentBody == null;
        }

        private void MjInitialize()
        {
            mass = mjBody.GetMass();
            inertiaLocalPos = mjBody.GetLocalCenterOfMass();
            inertiaRelMatrix = mjBody.GetInertiaToBodyMatrix();
        }

        public Vector3 Velocity => mjBody.GlobalVelocity();

        public Vector3 AngularVelocity => mjBody.GlobalAngularVelocity();

        public float Mass => mass;

        public Vector3 CenterOfMass => mjBody.GetTransformMatrix().MultiplyPoint3x4(inertiaLocalPos);  // TODO: Replace using Data->xipos

        public Vector3 Position => mjBody.GetPosition();

        public string Name => mjBody.name;

        public Matrix4x4 TransformMatrix => mjBody.GetTransformMatrix() * inertiaRelMatrix;

        //public Matrix4x4 LocalTransformMatrix => parentAdapter.TransformMatrix.inverse * TransformMatrix;

        public Vector3 Forward => mjBody.GetTransformMatrix().GetColumn(2);

        public Vector3 GetPointVelocity(Vector3 worldPoint)
        {
            return Vector3.Cross((worldPoint - CenterOfMass), AngularVelocity) + Velocity;
        }

        /// <summary>
        /// Follows Unity convention of methods with the same name
        /// </summary>
        /// <param name="localPoint">Point local to the inertial frame of the body (i.e. with the CoM in the centre)</param>
        /// <returns>Global velocity of the point</returns>
        public Vector3 GetRelativePointVelocity(Vector3 localPoint)
        {
            return Vector3.Cross(TransformMatrix.MultiplyVector(localPoint), AngularVelocity) + Velocity;
        }

        public Quaternion Rotation => MjState.GlobalRotation(mjBody);
        public Quaternion LocalRotation => isRoot ? Rotation : Quaternion.Inverse(MjState.GlobalRotation(parentBody)) * Rotation;

        public GameObject gameObject { get => mjBody.gameObject; }
        public MjBody ParentBody { get => parentBody; }

        public Vector3 LocalVelocity => MjState.LocalVelocity(mjBody);

        public Vector3 LocalAngularVelocity => MjState.LocalAngularVelocity(mjBody);
    }
}
