
using ModularAgents.Kinematic;
using ModularAgents.Kinematic.Mujoco;
using ModularAgents.MotorControl;
using Mujoco;
using Mujoco.Extensions;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.InteropServices.WindowsRuntime;
using UnityEditor.Animations;
using UnityEngine;
using UnityEngine.UIElements;

public class MjFiniteDifferenceJoint : MonoBehaviour, IFiniteDifferenceComponent, IMjJointStateProvider
{
    [SerializeField]
    MjBaseJoint pairedJoint;

    public MjBaseJoint PairedJoint { get=> pairedJoint; set => pairedJoint = value; }

    IMjJointState jointState;

    protected Quaternion initialRotationBody = Quaternion.identity;
    protected Quaternion initialRotationJoint = Quaternion.identity;

    public IMjJointState GetJointState()
    {
        if(jointState == null) 
            jointState = FiniteDifferenceJointState.GetFiniteDifferenceJointState(this, pairedJoint);
        return jointState;
    }


    private void Start()
    {
        //CheckLocalAxisPos();


        initialRotationBody =  pairedJoint.transform.parent.localRotation; //this seems to have fixed the lowerback


        //initialRotationBody = Quaternion.Inverse (pairedJoint.transform.localRotation) *pairedJoint.transform.parent.rotation; //this seems to have fixed the lowerback

          initialRotationJoint = pairedJoint.transform.localRotation;
        //initialRotationBody = pairedJoint.transform.localRotation * pairedJoint.transform.parent.localRotation ;
    }


    public void Step()
    {
        //I'm not sure we need to do anything here; it might be enough to step the body kinematics, then the joint components can remain largely stateless views into the body information.
    }
    private void OnDrawGizmosSelected()
    
    {

        DrawLocalRotations();


    }

    public void Update()
    {
        CheckRotations2Draw();
    }


    public void DrawLocalRotations()
    {
        Gizmos.color = Color.blue;


        if (pairedJoint != null)
        {
            double[] temp = GetJointState().Positions;


            if (temp.Length == 4) //its a ball joint
            { 

                //the localRotation of the paired joint is:
                //Quaternion FDLocalRotation = MjEngineTool.UnityQuaternion (new Quaternion((float) temp[0], (float) temp[1], (float) temp[2], (float) temp[3]));//the rotation of the parent

                Quaternion FDLocalRotation = new Quaternion((float)temp[1], (float)temp[3], (float)temp[2], -(float)temp[0]);


                Gizmos.DrawRay(transform.position, FDLocalRotation * Vector3.forward * 0.15f);
                Gizmos.color = Color.green;
                Gizmos.DrawRay(transform.position, FDLocalRotation * Vector3.up * 0.15f);
                Gizmos.color = Color.red;
                Gizmos.DrawRay(transform.position, FDLocalRotation * Vector3.right * 0.15f);

                Vector3 offset4debug = new Vector3(0, 0, 0.0f);


                if(Application.isPlaying )
                {

                    double[] temp2 = pairedJoint.GetQPos();
                                      
                    Quaternion MjLocalRotation =  new Quaternion((float)temp2[1], (float)temp2[3], (float)temp2[2], -(float)temp2[0])  ;
                 
                    Gizmos.color = Color.cyan;
                    Gizmos.DrawRay(transform.position + offset4debug, MjLocalRotation * Vector3.forward * 0.10f);
                    Gizmos.color = Color.yellow;
                    Gizmos.DrawRay(transform.position + offset4debug, MjLocalRotation * Vector3.up * 0.10f);
                    Gizmos.color = Color.grey;
                    Gizmos.DrawRay(transform.position + offset4debug, MjLocalRotation * Vector3.right * 0.10f);




                }





            }
        }
    }



    public void CheckRotations2Draw()
    {
       


        if (pairedJoint != null)
        {
            double[] temp = GetJointState().Positions;


            if (temp.Length == 4) //its a ball joint
            {

                //the localRotation of the paired joint is:
                Quaternion FDLocalRotation = new Quaternion((float)temp[1], (float)temp[3], (float)temp[2], -(float)temp[0]);//the rotation of the parent
                
                //Gizmos.DrawRay(transform.position, FDLocalRotation * Vector3.forward * 0.15f);
                //Gizmos.color = Color.green;
                //Gizmos.DrawRay(transform.position, FDLocalRotation * Vector3.up * 0.15f);
                //Gizmos.color = Color.red;
                //Gizmos.DrawRay(transform.position, FDLocalRotation * Vector3.right * 0.15f);

                Vector3 offset4debug = new Vector3(0, 0, 0.0f);

                Quaternion q2heck = transform.parent.localRotation;

                //Gizmos.color = Color.cyan;
                //Gizmos.DrawRay(transform.position + offset4debug, transform.parent.localRotation * Vector3.forward * 0.10f);
                //Gizmos.color = Color.yellow;
                //Gizmos.DrawRay(transform.position + offset4debug, transform.parent.localRotation * Vector3.up * 0.10f);
                //Gizmos.color = Color.grey;
                //Gizmos.DrawRay(transform.position + offset4debug, transform.parent.localRotation * Vector3.right * 0.10f);


                double[] temp2 = pairedJoint.GetQPos();


            }
        }
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


    

        /*
        var check = (MjHingeJoint)PairedJoint;
        if (check != null)
        {
            Vector3 ax = ((MjHingeJoint)PairedJoint).RotationAxis;
        }*/

            // Debug.Log(gameObject.name + " has up in unity: " + upUn + " up in mujoco: " + upMj + "  and local mj pos: " + mjpos + " rotation axis: " + ax);

       IMjJointState imj= GetJointState();
       Debug.Log(imj.Name + "   " + gameObject.name   + " has vels: " + imj.Velocities +  " has LOCAL ROTATIONS: " + imj.Positions   );
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

            //   Quaternion temp = Quaternion.Inverse(hinge.transform.localRotation) *  new Quaternion(LocalRotation.x, 0, 0, LocalRotation.w).normalized;
            // Quaternion temp =  new Quaternion(LocalRotation.x, 0, 0, LocalRotation.w).normalized;
            //Quaternion temp = new Quaternion(hinge.transform.localRotation.x, 0, 0, hinge.transform.localRotation.w).normalized;

            Quaternion temp = Quaternion.Inverse(hinge.transform.localRotation) * LocalRotation;
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

            // Vector3 temp = Quaternion.Inverse(ball.transform.localRotation) * LocalAngularVelocity;
            

            Vector3 temp = Quaternion.Inverse(component.initialRotationBody) * LocalAngularVelocity ;
            return new double[3] { temp.x, temp.z, temp.y };

        }

        double[] getJointLocalRotation()
        {

            //TODO: fix this, its wrong

            //Quaternion localJointRotation = gameObject.transform.rotation * LocalRotation;
            //Quaternion localJointRotation = Quaternion.Inverse(ball.transform.localRotation) * LocalRotation;
            //Quaternion localJointRotation = Quaternion.Inverse(ball.transform.localRotation) * LocalRotation;


            // double[] temp =((FiniteDifferenceJoint) this).GetJointState().Positions;


            // Quaternion FDLocalRotation = new Quaternion((float)temp[1], (float)temp[3], (float)temp[2], -(float)temp[0]);





            // return new double[4] {  - LocalRotation.w,                 LocalRotation.x,                 LocalRotation.z,                LocalRotation.y };


            Quaternion localJointRotation = parentKinematics.LocalRotation * Quaternion.Inverse(component.initialRotationBody); //the equivalent in MjBody: ball.transform.parent.GetIKinematic().LocalRotation;


//            Quaternion localJointRotation =  LocalRotation;

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