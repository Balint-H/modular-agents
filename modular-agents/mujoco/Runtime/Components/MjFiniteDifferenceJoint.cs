
using ModularAgents.Kinematic;
using ModularAgents.Kinematic.Mujoco;
using ModularAgents.MotorControl;
using Mujoco;
using Mujoco.Extensions;
using System.Collections.Generic;
using System.Linq;

using UnityEngine;


namespace Mujoco
{
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

    public class MjFiniteDifferenceJoint : MonoBehaviour, IFiniteDifferenceComponent, IMjJointStateProvider
    {


        [SerializeField]
        MjBaseJoint pairedJoint;

        public MjBaseJoint PairedJoint { get => pairedJoint; set => pairedJoint = value; }

        IMjJointState jointState;

        protected Quaternion initialRotationBody = Quaternion.identity;
        protected Quaternion initialRotationJoint = Quaternion.identity;


        /*
          
        //if we wanted this to inherit from MujocoBaseBody, we would do:
        protected override void OnParseMjcf(XmlElement mjcf) { }

        protected override XmlElement OnGenerateMjcf(XmlDocument doc)
        {
            return (XmlElement)doc.CreateElement("");
        }
        */

        //TO DEBUG:
        /*

        [Header("values to read:")]

        public float anglePositionDifference = 0;
        public Vector3 axisPositionDifference = Vector3.zero;

        public Quaternion FDLocalRotation = Quaternion.identity;
        public Quaternion PairedJointLocalRot = Quaternion.identity;
        */

        public IMjJointState GetJointState()
        {
            if (jointState == null)
                jointState = FiniteDifferenceJointState.GetFiniteDifferenceJointState(this, pairedJoint);
            return jointState;
        }


        private void Start()
        {



            MjState.ExecuteAfterMjStart(MjInitialize);



        }

        void MjInitialize()
        {
            //CheckLocalAxisPos();

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
            // if (name.Contains("lclavicle"))
            //     Debug.Log("lclavicle positions: " + "joint: " + name + "  " );


            double[] ps = GetJointState().Positions;

            // if (name.Contains("lclavicle"))
            //     Debug.Log("lclavicle positions: " +  "joint: " + name + "  " +  ps[0]);

            for (int i = 0; i < ps.Length; i++)
            {
                MjScene.Instance.Data->qpos[pairedJoint.QposAddress + i] = ps[i];
            }

            double[] vs = GetJointState().Velocities;

            for (int i = 0; i < vs.Length; i++)
            {
                MjScene.Instance.Data->qvel[pairedJoint.DofAddress + i] = vs[i];
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

        //To Debug:
        /*
        public void CheckRotations2Draw()
        {



            if (pairedJoint != null)
            {
                double[] temp = GetJointState().Positions;


                if (temp.Length == 4) //its a ball joint
                {


                    //the localRotation of the paired joint is:

                    FDLocalRotation =  new Quaternion((float)temp[1], (float)temp[3], (float)temp[2], -(float)temp[0]);
                    double[] temp2 = pairedJoint.GetQPos();

                    PairedJointLocalRot = new Quaternion((float)temp2[1], (float)temp2[3], (float)temp2[2], -(float)temp2[0]);

                    Debug.Log("on joint " + name + " we have: " + FDLocalRotation + " and: " + PairedJointLocalRot);

                    //(Quaternion.Inverse(pairedJointLocalRot) * FDLocalRotation).ToAngleAxis(out anglePositionDifference, out axisPositionDifference);




                }
            }
        }
        */



        public void CheckLocalAxisPos()
        {
            Quaternion q = MjEngineTool.MjQuaternion(gameObject.transform.rotation);

            //Vector3 upMj = MjEngineTool.MjVector3Up;
            Vector3 upMj = MjEngineTool.MjVector3(gameObject.transform.up);

            Vector3 upUn = gameObject.transform.up;
            //Vector3 test = LocalAngularVelocity;
            Vector3 mjpos = MjEngineTool.MjVector3(gameObject.transform.position);


            IMjJointState imj = GetJointState();
            Debug.Log(imj.Name + "   " + gameObject.name + " has vels: " + imj.Velocities + " has LOCAL ROTATIONS: " + imj.Positions);
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
            // Vector3 axis;
            MjHingeJoint hinge;

            MjFiniteDifferenceJoint[] siblings;

            int indexme = -1;

            public FiniteDifferenceHinge(MjFiniteDifferenceJoint component, MjHingeJoint hinge) : base(component)
            {
                //   axis = hinge.transform.localRotation * Vector3.right;

                this.hinge = hinge;

                MjFiniteDifferenceBody check = component.transform.parent.GetComponent<MjFiniteDifferenceBody>();

                siblings = check.GetBodyChildComponents<MjFiniteDifferenceJoint>().ToArray();    
                indexme = siblings.TakeWhile(x => ! x.name.Equals(component.transform.name)).Count() ;

                //if (siblings.Length >1) 
                //    Debug.Log("hinge joint: " + component.transform.name + "has index: " + indexme + " from a total number of hinges: " + siblings.Length + " First is: " + siblings[0].name + " Second is: " + siblings[1].name);

            }



            static Quaternion GetHingeRotation(MjFiniteDifferenceJoint component, MjHingeJoint hinge)
            {
                FiniteDifferenceHinge fdh = new FiniteDifferenceHinge(component, hinge);

                Quaternion hingeRot = (Quaternion.Inverse(fdh.component.transform.localRotation) * fdh.LocalRotation);

                //the rotation that we are interested in corresponds to the X component:
                //return (new Quaternion(hingeRot.x, 0,0, hingeRot.w).normalized);
                return hingeRot;


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

                return new double[1] { (Quaternion.Inverse(component.transform.localRotation) * LocalAngularVelocity).x };



            }

            double[] GetJointLocalRotation()
            {
                // Quaternion temp = Quaternion.Inverse(hinge.transform.localRotation) * LocalRotation; //this works with hte pupeteer but not for the pure FD case (because when resetting the hinge the orientaiton might not match??)
                switch (siblings.Length)
                {
                    case 0:
                        Debug.LogWarning("Trying to get the location of a Hinge but the MjJoint" + component.transform.localRotation + " has no Hinger attached to it. This shouldnt be possible");
                        return new double[1] { 0.0 };
                      

                    case 1:
                        {
                            Quaternion temp = Quaternion.Inverse(component.transform.localRotation) * LocalRotation;
                            return new double[1] { 2 * Mathf.Asin(temp.x) * Mathf.Sign(-temp.w) };  //this is what would be mathematically correct when reverting a quaternion to angles

                        }

                    default: //tested only for 2 elements, the case with 3 hinges is not considered for now.
                        {

                            if (indexme == 0)
                            {
                                Quaternion temp = Quaternion.Inverse(component.transform.localRotation) * LocalRotation;
                                return new double[1] { 2 * Mathf.Asin(temp.x) * Mathf.Sign(-temp.w) };
                            }
                            else
                            { 
                            Quaternion[] resultsToCheck = new Quaternion[siblings.Length];


                            resultsToCheck = siblings.Select(x => FiniteDifferenceHinge.GetHingeRotation(x, hinge)).ToArray();

                                FiniteDifferenceHinge[] hinges = siblings.Select(x => new FiniteDifferenceHinge(x, hinge)).ToArray();

                                Quaternion temp = Quaternion.Inverse(component.transform.localRotation) * Quaternion.Inverse(hinges[0].component.transform.localRotation) * hinges[0].LocalRotation;

                                //Quaternion composedRot = resultsToCheck[1] * resultsToCheck[0];

                           // Debug.Log( "index: "+indexme+ " component:" + component.transform.name +   "referenceRot: " + Quaternion.Inverse(component.transform.localRotation) * LocalRotation);

                            //Quaternion temp2 = Quaternion.Inverse(component.transform.localRotation) * LocalRotation;
                            return new double[1] { 2 * Mathf.Asin(temp.x) * Mathf.Sign(-temp.w) };  //this is what would be mathematically correct when reverting a quaternion to angles

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



}