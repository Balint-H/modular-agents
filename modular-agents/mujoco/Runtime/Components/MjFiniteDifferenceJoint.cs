
using ModularAgents.Kinematic;
using ModularAgents.Kinematic.Mujoco;
using ModularAgents.MotorControl;
using Mujoco;
using Mujoco.Extensions;
using System.Collections.Generic;
using System.Linq;

using UnityEngine;


using ModularAgents;

    public class MjFiniteDifferenceJoint : MonoBehaviour, IFiniteDifferenceComponent, IMjJointStateProvider
    {


        [SerializeField]
        MjBaseJoint pairedJoint;

        public MjBaseJoint PairedJoint { get => pairedJoint; set => pairedJoint = value; }

        IMjJointState jointState;

        protected Quaternion initialRotationBody = Quaternion.identity;


        FiniteDifferenceHinge hingeFD = null; 

        public Vector3 HingeRotationAxis
        {


            get { if (hingeFD != null)

                    return hingeFD.RotationAxis ;
                else
                    return Vector3.zero;


            }

        }


    public int HingeIndex
    {


        get
        {
            if (hingeFD != null)

                return hingeFD.IndexMe;
            else
                return -1;


        }

    }






    /*

    //if we wanted this to inherit from MujocoBaseBody, we would do:
    protected override void OnParseMjcf(XmlElement mjcf) { }

    protected override XmlElement OnGenerateMjcf(XmlDocument doc)
    {
        return (XmlElement)doc.CreateElement("");
    }
    */

    public double[] QPos
         {
        get {return  GetJointState().Positions; }
    
         }

        public double[] QVel
        {
            get { return GetJointState().Velocities; }

        }



    public IMjJointState GetJointState()
        {
            if (jointState == null)
            {
                jointState = FiniteDifferenceJointState.GetFiniteDifferenceJointState(this, pairedJoint);
                if(jointState.Positions.Length == 1) //if its a hinge we need to keep track of the rotation axis for display
                    hingeFD = new FiniteDifferenceHinge(this, (MjHingeJoint) pairedJoint);


            }
            
            return jointState;
        }


        private void Start()
        {



            MjState.ExecuteAfterMjStart(MjInitialize);



        }

        void MjInitialize()
        {
           
            initialRotationBody = pairedJoint.transform.parent.localRotation;
           
         
        }



        public void Step()
        {
            //I'm not sure we need to do anything here; it might be enough to step the body kinematics, then the joint components can remain largely stateless views into the body information.
        }
        private void OnDrawGizmosSelected()

        {

            DrawLocalRotations();


        }

        public unsafe void Reset()
        {
           

            double[] ps = GetJointState().Positions;

         
            for (int i = 0; i < ps.Length; i++)
            {
                MjScene.Instance.Data->qpos[pairedJoint.QposAddress + i] = ps[i];
            }

            double[] vs = GetJointState().Velocities;

            for (int i = 0; i < vs.Length; i++)
            {
                MjScene.Instance.Data->qvel[pairedJoint.DofAddress + i] = vs[i];
            }

   
            for (int i = 0; i < vs.Length; i++)
            {
                MjScene.Instance.Data->qfrc_applied[pairedJoint.DofAddress + i] = 0;
            }

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


                    if (Application.isPlaying)
                    {

                        double[] temp2 = pairedJoint.GetQPos();

                        Quaternion MjLocalRotation = new Quaternion((float)temp2[1], (float)temp2[3], (float)temp2[2], -(float)temp2[0]);

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

            public string Name => component.name + "_JointState";

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
        MjHingeJoint hinge;
        Quaternion initialRotationHinge;



        MjFiniteDifferenceJoint[] siblings;

        public FiniteDifferenceHinge[] HingeSiblings
            {   
                get =>  siblings.Select(x => new FiniteDifferenceHinge(x, (MjHingeJoint) x.pairedJoint) ).ToArray();

            }

      
        Quaternion[] initialRotationSiblings;


        Transform referenceBodyTransform; //for the first hinge, it is the body grand parent, for the second and the rest of hinges, it is the previous hinge
        Quaternion deviationFromReferenceBody;

  
        MjFiniteDifferenceBody bodyIAmTurning;

        public Vector3 RotationAxis {
            get => referenceBodyTransform.transform.rotation * deviationFromReferenceBody * Vector3.right;
        }


      
        int indexme = -1;

        public int IndexMe { get => indexme; }

        public FiniteDifferenceHinge(MjFiniteDifferenceJoint component, MjHingeJoint hinge) : base(component)
        {


            this.hinge = hinge;
            initialRotationHinge = hinge.transform.localRotation;


            bodyIAmTurning = component.transform.parent.GetComponent<MjFiniteDifferenceBody>();

            siblings = bodyIAmTurning.GetBodyChildComponents<MjFiniteDifferenceJoint>().ToArray();
            indexme = siblings.TakeWhile(x => !x.name.Equals(component.transform.name)).Count();
            initialRotationSiblings = siblings.Select(x => x.GetComponent<MjFiniteDifferenceJoint>().pairedJoint.transform.localRotation).ToArray();
            //initialRotationSiblings = siblings.Select(x => x.GetComponent<MjFiniteDifferenceJoint>().transform.localRotation).ToArray();


            if (indexme == 0)
                //referenceBodyTransform = bodyIAmTurning.GetComponentInParent<MjFiniteDifferenceBody>().transform; //rotations are relative to the grand-dad
                referenceBodyTransform = bodyIAmTurning.transform.parent.GetComponent<MjFiniteDifferenceBody>().transform; //rotations are relative to the grand-dad

            else
                referenceBodyTransform = siblings[indexme - 1].transform;

            //deviationFromReferenceBody =  hinge.transform.rotation * Quaternion.Inverse(referenceBodyTransform.transform.rotation);
            deviationFromReferenceBody =  Quaternion.Inverse(referenceBodyTransform.transform.rotation) * hinge.transform.rotation ;



        }



        public int[] DofAddresses => throw new System.NotImplementedException();  // Okay to leave as such, or replace with negative values

        public int[] PosAddresses => throw new System.NotImplementedException();  // Okay to leave as such, or replace with negative values

        public double[] TargetPositions => throw new System.NotImplementedException();  // Okay to leave as such, or replace with zero values

        public MjBaseJoint Joint => hinge;

        public IMjJointState ReferenceState => throw new System.NotImplementedException(); //  Okay to leave as such, or replace with zero class

        public double[] Accelerations => throw new System.NotImplementedException("Currently only first order joint states supported.");


        public double[] Velocities => GetJointAngularVelocity();


        double[] GetJointAngularVelocity()
        {



            return new double[1] { (Quaternion.Inverse(hinge.transform.localRotation) * LocalAngularVelocity).x };



        }


        Quaternion GetBodyLocalRotation()
        {



            Quaternion localJointRotation = Quaternion.Inverse(component.initialRotationBody) * parentKinematics.LocalRotation;

            return localJointRotation;


        }


        double[] GetJointLocalRotation()
        {

            //A. version with no siblings:

            //Quaternion temp = Quaternion.Inverse(initialRotationHinge) * GetBodyLocalRotation();
            //return new double[1] { 2 * Mathf.Asin(temp.x) * Mathf.Sign(-temp.w) };


            //return new double[1] { Quaternion.Angle(component.transform.rotation , initGlobalRot ) * Mathf.Deg2Rad };

            //B. version with 1 sibling:    
            //below, an attempt to take into account the different hinges that are s
            
            switch (siblings.Length)
            {
                case 0:
                    Debug.LogWarning("Trying to get the location of a Hinge but the MjJoint" + component.transform.localRotation + " has no Hinge attached to it. This shouldnt be possible");
                    return new double[1] { 0.0 };


                case 1:
                    {

                        Quaternion temp = Quaternion.Inverse(initialRotationHinge) * GetBodyLocalRotation();


                        // parentKinematics.    parentKinematics.LocalRotation;

                        return new double[1] { 2 * Mathf.Asin(temp.x) * Mathf.Sign(-temp.w) };  //this is what would be mathematically correct when reverting a quaternion to angles

                    }

                default: //tested only for 2 elements, the case with 3 hinges is not considered for now.
                    {

                        if (indexme == 1)
                        {


                            Quaternion temp = Quaternion.Inverse(initialRotationSiblings[indexme]) * GetBodyLocalRotation();

                            return new double[1] { 2 * Mathf.Asin(temp.x) * Mathf.Sign(-temp.w) };

                        }


                        else //indexme is 0
                        {

                            //WRONG

                            
                            Quaternion temp = Quaternion.Inverse(initialRotationSiblings[0]) * GetBodyLocalRotation();
                            Quaternion meQ = new Quaternion(temp.x, 0, 0, temp.w).normalized;
                            Quaternion temp2 = Quaternion.Inverse(meQ) * Quaternion.Inverse(initialRotationSiblings[1]) * GetBodyLocalRotation();

                            return new double[1] { -2 * Mathf.Asin(temp2.x) * Mathf.Sign(-temp2.w) };

                            //Quaternion temp2 = Quaternion.Inverse(initialRotationSiblings[0]) * Quaternion.Inverse(initialRotationSiblings[1]) * GetBodyLocalRotation();
                            

                            //Quaternion temp3 = component.initialRotationBody * Quaternion.Inverse(parentKinematics.LocalRotation) * initialRotationSiblings[0];

                           // return new double[1] { -2 * Mathf.Asin(temp3.x) * Mathf.Sign(-temp3.w) };



                        }


                    }

            }




        }




        public double[] Positions => GetJointLocalRotation();

        public void CheckLocalRotations()
        {
            Quaternion q = MjEngineTool.MjQuaternion(gameObject.transform.rotation);

            //Vector3 upMj = MjEngineTool.MjVector3Up;
            Vector3 upMj = MjEngineTool.MjVector3(gameObject.transform.up);

            Vector3 upUn = gameObject.transform.up;
            Vector3 test = LocalAngularVelocity;


            Debug.Log(gameObject.name + " has up in unity: " + upUn + " up in mujoco: " + upMj + "  and local angle velocity: " + test);

        }





        //public double[] PositionErrors => throw new System.NotImplementedException();  // TODO (Positions for hinge, quat error from identity for ball)

        public double[] PositionErrors => Positions.Select(x => -x).ToArray();//reference (here 0)minus me

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


            public double[] Velocities => GetJointAngularVelocity();
            public double[] Positions => GetJointLocalRotation();



            double[] GetJointAngularVelocity()
            {


                Vector3 temp = Quaternion.Inverse(component.initialRotationBody) * LocalAngularVelocity;
                return new double[3] { temp.x, temp.z, temp.y };

            }

            double[] GetJointLocalRotation()
            {


                Quaternion localJointRotation = Quaternion.Inverse(component.initialRotationBody) * parentKinematics.LocalRotation;

                //in mujoco coordinates, this gives:
                //return new double[4] { -localJointRotation.w, localJointRotation.x, localJointRotation.z, localJointRotation.y };
                if (localJointRotation.w > 0)
                    return new double[4] { localJointRotation.w, -localJointRotation.x, -localJointRotation.z, -localJointRotation.y };
                return new double[4] { -localJointRotation.w, localJointRotation.x, localJointRotation.z, localJointRotation.y };


        }


        public double[] PositionErrors => Utils.QuaternionError(Positions,new double[4] {1,0,0,0 });//the difference between the current position and the null rotatoin in mujoco space
            

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



            public double[] Velocities => GetVelocities();
            public double[] Positions => GetLocalRotation();


            double[] GetVelocities()
            {

                //in unity coordinates:
                //return new double[6] { parentKinematics.Velocity.x, parentKinematics.Velocity.y, parentKinematics.Velocity.z,
                //                        LocalAngularVelocity.x,     LocalAngularVelocity.y,      LocalAngularVelocity.z };

                //in Mujoco:
                return new double[6] { parentKinematics.Velocity.x, parentKinematics.Velocity.z, parentKinematics.Velocity.y,
                                    LocalAngularVelocity.x,     LocalAngularVelocity.z,      LocalAngularVelocity.y };
            }

            double[] GetLocalRotation()
            {

                //in Mujoco coordinates: (x,z,y) (-w, x,z,y)
                return new double[7] { parentKinematics.Position.x,      parentKinematics.Position.z,     parentKinematics.Position.y,
                                    - LocalRotation.w,                 LocalRotation.x,                 LocalRotation.z,                LocalRotation.y };//this is a global rotation, see in IKinematic

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

    public static class MjFD
    {
        public static IEnumerable<T> GetBodyChildComponents<T>(this MjFiniteDifferenceBody body) where T : MjFiniteDifferenceJoint
        {
            foreach (var childComponent in body.GetComponentsInChildren<T>().OrderBy(c => c.transform.GetSiblingIndex()))
            {
                if (MjHierarchyTool.FindParentComponent<MjFiniteDifferenceBody>(childComponent) == body) yield return childComponent;
            }
        }

    }
