
using ModularAgents.Kinematic;
using ModularAgents.Kinematic.Mujoco;
using ModularAgents.MotorControl;
using Mujoco;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices.WindowsRuntime;
using UnityEngine;

public class MjFiniteDifferenceJoint : MonoBehaviour, IFiniteDifferenceComponent, IMjJointStateProvider
{
    [SerializeField]
    MjBaseJoint pairedJoint;

    IMjJointState jointState;

    public IMjJointState GetJointState()
    {
        if(jointState == null) jointState = FiniteDifferenceJointState.GetFiniteDifferenceJointState(this, pairedJoint);
        return jointState;
    }

    public void Step()
    {
        //I'm not sure we need to do anything here; it might be enough to step the body kinematics, then the joint components can remain largely stateless views into the body information.
    }

    private abstract class FiniteDifferenceJointState
    {
        protected MjFiniteDifferenceJoint component;
        protected IKinematic parentKinematics;

        protected Quaternion LocalRotation => parentKinematics.LocalRotation;
        protected Vector3 LocalAngularVelocity => parentKinematics.LocalAngularVelocity;

        public FiniteDifferenceJointState(MjFiniteDifferenceJoint component)
        {
            this.component = component;
            parentKinematics = component.GetComponentInParent<MjFiniteDifferenceBody>().GetIKinematic();
        }

        public string Name => component.name+"_JointState";

        public GameObject gameObject => component.gameObject;

        public static IMjJointState GetFiniteDifferenceJointState(MjFiniteDifferenceJoint component, MjBaseJoint pairedJoint)
        {
            switch (pairedJoint)
            {
                case MjHingeJoint hinge:
                    return new FiniteDifferenceHinge(component, hinge);
                case MjBallJoint ball:
                    return new FiniteDifferenceBall(component, ball);
                case MjFreeJoint freeJoint:
                    return new FiniteDifferenceFreeJoint(component, freeJoint);
            }
            throw new System.NotImplementedException($"Joint {pairedJoint.name}'s joint type is not yet supported for finite difference kinematics.");
        }
    }

    private class FiniteDifferenceHinge : FiniteDifferenceJointState, IMjJointState
    {
        Vector3 axis;
        MjHingeJoint hinge;

        public FiniteDifferenceHinge(MjFiniteDifferenceJoint component, MjHingeJoint hinge) : base(component)
        {
            axis = hinge.RotationAxis;
            this.hinge = hinge;
        }

        public int[] DofAddresses => throw new System.NotImplementedException();  // Okay to leave as such, or replace with negative values

        public int[] PosAddresses => throw new System.NotImplementedException();  // Okay to leave as such, or replace with negative values

        public double[] TargetPositions => throw new System.NotImplementedException();  // Okay to leave as such, or replace with zero values

        public MjBaseJoint Joint => hinge;

        public IMjJointState ReferenceState => throw new System.NotImplementedException(); //  Okay to leave as such, or replace with zero class

        public double[] Accelerations => throw new System.NotImplementedException("Currently only first order joint states supported.");

        public double[] Velocities => throw new System.NotImplementedException();  // TODO 

        public double[] Positions => throw new System.NotImplementedException();  // TODO
        //check into MjEngineTool.MjQuaternion for conversion

        public double[] PositionErrors => throw new System.NotImplementedException();  // TODO (Positions for hinge, quat error from identity for ball)

        public double[] VelocityErrors => Velocities;

        public double[] GetStablePositionErrors(double dt)
        {
            throw new System.NotImplementedException();  // Okay to leave as such
        }

        public double[] GetStableVelocityErrors(double dt)
        {
            throw new System.NotImplementedException();  // Okay to leave as such
        }
    }

    private class FiniteDifferenceBall : FiniteDifferenceJointState, IMjJointState
    {
        MjBallJoint ball;
        public FiniteDifferenceBall(MjFiniteDifferenceJoint component, MjBallJoint ball) : base(component)
        {
            this.ball = ball;
        }

        public int[] DofAddresses => throw new System.NotImplementedException();

        public int[] PosAddresses => throw new System.NotImplementedException();

        public double[] TargetPositions => throw new System.NotImplementedException();

        public MjBaseJoint Joint => throw new System.NotImplementedException();

        public IMjJointState ReferenceState => throw new System.NotImplementedException();

        public double[] Accelerations => throw new System.NotImplementedException();

        public double[] Velocities => throw new System.NotImplementedException();

        public double[] Positions => throw new System.NotImplementedException();

        public double[] PositionErrors => throw new System.NotImplementedException();

        public double[] VelocityErrors => throw new System.NotImplementedException();

        public double[] GetStablePositionErrors(double dt)
        {
            throw new System.NotImplementedException();
        }

        public double[] GetStableVelocityErrors(double dt)
        {
            throw new System.NotImplementedException();
        }
    }

    private class FiniteDifferenceFreeJoint : FiniteDifferenceJointState, IMjJointState
    {
        MjFreeJoint freeJoint;
        public FiniteDifferenceFreeJoint(MjFiniteDifferenceJoint component, MjFreeJoint freeJoint) : base(component)
        {
            this.freeJoint = freeJoint;
        }

        public int[] DofAddresses => throw new System.NotImplementedException();

        public int[] PosAddresses => throw new System.NotImplementedException();

        public double[] TargetPositions => throw new System.NotImplementedException();

        public MjBaseJoint Joint => throw new System.NotImplementedException();

        public IMjJointState ReferenceState => throw new System.NotImplementedException();

        public double[] Accelerations => throw new System.NotImplementedException();

        public double[] Velocities => throw new System.NotImplementedException();

        public double[] Positions => throw new System.NotImplementedException();

        public double[] PositionErrors => throw new System.NotImplementedException();

        public double[] VelocityErrors => throw new System.NotImplementedException();

        public double[] GetStablePositionErrors(double dt)
        {
            throw new System.NotImplementedException();
        }

        public double[] GetStableVelocityErrors(double dt)
        {
            throw new System.NotImplementedException();
        }
    }
}

public interface IMjJointStateProvider
{
    IMjJointState GetJointState();
}