
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

    public MjBaseJoint PairedJoint { get=> pairedJoint; set => pairedJoint = value; }

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

    public void CheckLocalAxisPos()
    {
        Quaternion q = MjEngineTool.MjQuaternion(gameObject.transform.rotation);

        //Vector3 upMj = MjEngineTool.MjVector3Up;
        Vector3 upMj = MjEngineTool.MjVector3(gameObject.transform.up);

        Vector3 upUn = gameObject.transform.up;
        //Vector3 test = LocalAngularVelocity;
        Vector3 mjpos = MjEngineTool.MjVector3(gameObject.transform.position);
        //return new double[3] { mjpos.x, mjpos.y, mjpos.z };

        Vector3 ax = ((MjHingeJoint)PairedJoint).RotationAxis;


            // Debug.Log(gameObject.name + " has up in unity: " + upUn + " up in mujoco: " + upMj + "  and local mj pos: " + mjpos + " rotation axis: " + ax);

       IMjJointState imj= GetJointState();
       Debug.Log(gameObject.name + " has vels: " + imj.Velocities + " has LOCAL ROTATIONS: " + imj.Positions);
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

        //  public double[] Velocities => throw new System.NotImplementedException();  // TODO 

        public double[] Velocities => GetLocalAngularVelocity();


        double[] GetLocalAngularVelocity()
        {
           return  new double[3] { LocalAngularVelocity.x, 0, 0 };

        }

        double[] GetLocalRotation()
        {

            Quaternion temp = new Quaternion(LocalRotation.x, 0, 0, LocalRotation.w).normalized;
            return new double[3] { Mathf.Asin( 2* temp.x), 0, 0 };

        }
        

        //public double[] Positions => throw new System.NotImplementedException();   // TODO

        public double[] Positions => GetLocalRotation();


        //check into MjEngineTool.MjQuaternion for conversion

      

        //   double[] getLocalRotations()
       public  void CheckLocalRotations()
        {
            Quaternion q = MjEngineTool.MjQuaternion(gameObject.transform.rotation);

            //Vector3 upMj = MjEngineTool.MjVector3Up;
            Vector3 upMj = MjEngineTool.MjVector3(gameObject.transform.up);

            Vector3 upUn = gameObject.transform.up;
            Vector3 test = LocalAngularVelocity;
            //Vector3 mjpos = MjEngineTool.MjVector3(gameObject.transform.position);
            //return new double[3] { mjpos.x, mjpos.y, mjpos.z };

            Debug.Log(gameObject.name + " has up in unity: " + upUn + " up in mujoco: " + upMj + "  and local angle velocity: " + test);

        }





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

       
        public double[] Velocities => getLocalAngularVelocity();
        public double[] Positions => getLocalRotation();



        double[] getLocalAngularVelocity()
        {
            return new double[3] { LocalAngularVelocity.x, LocalAngularVelocity.y, LocalAngularVelocity.z };

        }

        double[] getLocalRotation()
        {


            return new double[3] { Mathf.Asin(2 * LocalRotation.x), Mathf.Asin(2 * LocalRotation.y), Mathf.Asin(2 * LocalRotation.z) };

        }


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


    /*
      // Transform
      transform.localPosition =
          MjEngineTool.UnityVector3(mjcf.GetVector3Attribute("pos", defaultValue: Vector3.zero));
      transform.localRotation = MjEngineTool.UnityQuaternion(
          mjcf.GetQuaternionAttribute("quat", defaultValue: MjEngineTool.MjQuaternionIdentity));
      GravityCompensation = mjcf.GetFloatAttribute("gravcomp", defaultValue: 0.0f); 
     
     */


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
        /*
        public override unsafe void OnSyncState(MujocoLib.mjData_* data)
        {
            transform.position = MjEngineTool.UnityVector3(
                MjEngineTool.MjVector3AtEntry(data->xpos, MujocoId));
            transform.rotation = MjEngineTool.UnityQuaternion(
                MjEngineTool.MjQuaternionAtEntry(data->xquat, MujocoId));
        }*/

        public double[] Positions => throw new System.NotImplementedException();
        //public double[] Positions => MjEngineTool.MjVector3(transform) ;

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