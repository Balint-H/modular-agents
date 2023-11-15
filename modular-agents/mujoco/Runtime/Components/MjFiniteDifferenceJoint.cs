
using ModularAgents.Kinematic;
using ModularAgents.Kinematic.Mujoco;
using ModularAgents.MotorControl;
using Mujoco;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices.WindowsRuntime;
using UnityEditor.Animations;
using UnityEngine;

public class MjFiniteDifferenceJoint : MonoBehaviour, IFiniteDifferenceComponent, IMjJointStateProvider
{
    [SerializeField]
    MjBaseJoint pairedJoint;

    public MjBaseJoint PairedJoint { get=> pairedJoint; set => pairedJoint = value; }

    IMjJointState jointState;

    public IMjJointState GetJointState()
    {
        if(jointState == null) 
            jointState = FiniteDifferenceJointState.GetFiniteDifferenceJointState(this, pairedJoint);
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

        public double[] Velocities => GetJointAngularVelocity();


        double[] GetJointAngularVelocity()
        {
             //return new double[1] {  LocalAngularVelocity.x };

            return new double[1] { (Quaternion.Inverse(hinge.transform.localRotation) * LocalAngularVelocity).x };




        }

        double[] GetJointLocalRotation()
        {

            Quaternion temp = Quaternion.Inverse(hinge.transform.localRotation) *  new Quaternion(LocalRotation.x, 0, 0, LocalRotation.w).normalized;
            //Quaternion temp =  new Quaternion(LocalRotation.x, 0, 0, LocalRotation.w).normalized;
            return new double[1] { 2 * Mathf.Asin(temp.x) };

        }
        

    

        public double[] Positions => GetJointLocalRotation();


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

       
        public double[] Velocities => getJointAngularVelocity();
        public double[] Positions => getJointLocalRotation();



        double[] getJointAngularVelocity()
        {
            // return new double[3] { LocalAngularVelocity.x, LocalAngularVelocity.y, LocalAngularVelocity.z };
            //in Mujoco:

            Vector3 temp =  LocalAngularVelocity;

            return new double[3] { temp.x, temp.z, temp.y };

        }

        double[] getJointLocalRotation()
        {

            //TODO: fix this, its wrong
           
            //Quaternion localJointRotation = gameObject.transform.rotation * LocalRotation;
            Quaternion localJointRotation = Quaternion.Inverse(ball.transform.localRotation) * LocalRotation;


            //in mujoco coordinates, this gives:
            return new double[4] { -localJointRotation.w, localJointRotation.x, localJointRotation.z, localJointRotation.y };

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

       

        public double[] Velocities => getVelocities();
        public double[] Positions => getLocalRotation();


        double[] getVelocities()
        {

            //in unity coordinates:
            //return new double[6] { parentKinematics.Velocity.x, parentKinematics.Velocity.y, parentKinematics.Velocity.z,
            //                        LocalAngularVelocity.x,     LocalAngularVelocity.y,      LocalAngularVelocity.z };

            //in Mujoco:
            return new double[6] { parentKinematics.Velocity.x, parentKinematics.Velocity.z, parentKinematics.Velocity.y,
                                    LocalAngularVelocity.x,     LocalAngularVelocity.z,      LocalAngularVelocity.y };
        }

        double[] getLocalRotation()
        {

            //in unity coordinates it would be:
            //return new double[7] { parentKinematics.Position.x,      parentKinematics.Position.y,     parentKinematics.Position.z,
            //                        LocalRotation.x,                 LocalRotation.y,                 LocalRotation.z,                LocalRotation.w };//this is a global rotation due to IKinematic

            //in Mujoco coordinates: (x,z,y) (-w, x,z,y)
            return new double[7] { parentKinematics.Position.x,      parentKinematics.Position.z,     parentKinematics.Position.y,
                                    - LocalRotation.w,                 LocalRotation.x,                 LocalRotation.z,                LocalRotation.y };//this is a global rotation due to IKinematic

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
}

public interface IMjJointStateProvider
{
    IMjJointState GetJointState();
}