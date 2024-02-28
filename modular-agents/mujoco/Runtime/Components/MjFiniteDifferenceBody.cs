
using UnityEngine;
using Mujoco;

using Mujoco.Extensions;
using System;

namespace ModularAgents.Kinematic.Mujoco
{
    /// <summary>
    /// A MocapBody that can provide 6 DOF velocity estimates based on finite differences.
    /// </summary>

    public class MjFiniteDifferenceBody : MonoBehaviour, IKinematicProvider, IFiniteDifferenceComponent
    {
        [SerializeField]
        MjBody pairedBody;

        public MjBody PairedBody { get => pairedBody; set => pairedBody = value; }
   
        Vector3 prevPosition;

        Vector3 currentPosition;


        Quaternion prevRotation = Quaternion.identity;
        Quaternion currentRotation = Quaternion.identity;

        Quaternion prevLocalRotation  = Quaternion.identity;
        Quaternion currentLocalRotation = Quaternion.identity;

        public Vector3 Position => transform.position;
        public Quaternion Rotation =>  transform.rotation;


        // private Quaternion localRotationOffset = Quaternion.identity;

        // private Quaternion LocalRotation => Quaternion.Inverse(localRotationOffset) * transform.localRotation;
        
        private Quaternion LocalRotation =>  transform.localRotation;

        private float fs;
        private Vector3 Velocity => (currentPosition - prevPosition) * fs;

        public Vector3 AngularVelocity => Utils.RotationVel(currentRotation, prevRotation, fs);

        //we express it in its parent's coordinates:
        public Vector3 LocalAngularVelocity =>   Utils.RotationVel(currentLocalRotation, prevLocalRotation, fs);


        // notice we want this in the parent's coordinate axis, if we want to visualise in global cartesian coordinates we can simply add the parents rotation:
        //private Vector3 LocalAngularVelocityCartesianCoords => transform.parent.rotation * LocalAngularVelocity;


        [SerializeField]
        private Vector3 localForward = Vector3.forward;

        FiniteDifferenceBodyKinematics kinematics;

        public static
        Vector3 offset4debug = new Vector3(0.05f, 0.0f, 0.0f);




        private void Awake()
        {
            Step();
        }

        private void Start()
        {
            MjState.ExecuteAfterMjStart(MjInitialize);
        }

        private void MjInitialize()
        {
            fs = 1 / Time.fixedDeltaTime;
          


        }

      

        /*   public void FixedUpdate()
           {
               Step();
           }
        */

        public void Step()
        {
            prevPosition = currentPosition;
            currentPosition = transform.position;

            prevRotation = currentRotation;
            currentRotation = transform.rotation;

            prevLocalRotation = currentLocalRotation;
            currentLocalRotation = LocalRotation;
    

    }

    public IKinematic GetIKinematic()
        {
            if(kinematics == null) kinematics = new FiniteDifferenceBodyKinematics(this);
            return kinematics;
        }

        private void OnDrawGizmosSelected()
        {
            if (!Application.isPlaying) return;

            // DrawLocalRotations();

            //  DrawLocalAngularVelocities();

            // DrawNormalTangent();
            /*
            DebugValuesFor("lhumerus");
            DebugValuesFor("lowerback");

            DebugValuesFor("rtibia");
            */
        }

        public void DrawLocalAngularVelocities()
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(GetIKinematic().CenterOfMass, 0.01f);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(Position, Rotation * Vector3.forward * 0.015f);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(Position, Rotation * Vector3.up * 0.015f);
            Gizmos.color = Color.red;
            Gizmos.DrawRay(Position, Rotation * Vector3.right * 0.015f);




            

            Gizmos.color = Color.black;
            Gizmos.DrawRay(Position + 0.01f * Vector3.up, (transform.parent.rotation *LocalAngularVelocity * 0.1f));

            MjBody pupeteeredJoint4Debug = GetComponent<MjBody>();

            if (pupeteeredJoint4Debug != null)
            {      
                Gizmos.color = Color.blue;                    
                IKinematic pupetKin = pupeteeredJoint4Debug.transform.GetIKinematic();

                var parent = pupeteeredJoint4Debug.transform.parent.GetComponent<MjBody>();

                Gizmos.DrawRay(Position, parent.GetTransformMatrix().MultiplyVector(   pupetKin.LocalAngularVelocity )* 0.1f);

                
                Gizmos.color = Color.yellow;
                Gizmos.DrawRay(Position, pupetKin.AngularVelocity * 0.1f);

             

           
            }

            Gizmos.color = Color.red;
            Gizmos.DrawRay(Position + 0.005f * Vector3.up, AngularVelocity * 0.1f);


        }

        void Update()
        {

            DebugValuesFor("rfemur");

        }

        public void DebugValuesFor(string targetname)
        {

            if (name.Contains(targetname))
            {
                MjBody pupeteeredJoint4Debug = GetComponent<MjBody>();

                if (pupeteeredJoint4Debug == null)
                    return;

                IKinematic pupetKin = pupeteeredJoint4Debug.transform.GetIKinematic();
                Debug.Log( targetname +":          AngVel: " + AngularVelocity + "  pupet: " + pupeteeredJoint4Debug.GlobalAngularVelocity() + "pupetKin:" + pupetKin.AngularVelocity);
                Debug.Log(targetname + ":     localAngVel: " + LocalAngularVelocity + "  pupet: " + pupetKin.LocalAngularVelocity );
                // Debug.Log(":     localRot: " + LocalRotation + "  pupet: " + pupetKin.LocalRotation);
                //  var parent = pupeteeredJoint4Debug.transform.parent.GetComponent<MjBody>();
                //  Debug.Log(" Dad localRot: " + transform.parent.localRotation + "  pupet: dad " + parent.transform.GetIKinematic().LocalRotation +" is: " + parent.transform.GetIKinematic().Name);
                //  Debug.Log(" Dad globlRot: " + transform.parent.rotation + "  pupet: dad " + parent.GlobalRotation() + " is: " + parent.name);

               // Debug.Log(targetname + ":     globalVel: " + Velocity + "  pupet: " + pupeteeredJoint4Debug.GlobalVelocity()   + "pupetKin: " + pupetKin.Velocity);


            }


        }



        public void DrawLocalRotations()
        {
          //  Gizmos.color = Color.yellow;
          //  Gizmos.DrawWireSphere(GetIKinematic().CenterOfMass, 0.01f);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(Position, LocalRotation * Vector3.forward * 0.15f);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(Position, LocalRotation * Vector3.up * 0.15f);
            Gizmos.color = Color.red;
            Gizmos.DrawRay(Position, LocalRotation * Vector3.right * 0.15f);

            MjBody pupeteeredJoint4Debug = GetComponent<MjBody>();
            if (pupeteeredJoint4Debug != null)
            {
                IKinematic pupetKin = pupeteeredJoint4Debug.transform.GetIKinematic();
                Gizmos.color = Color.cyan;
                Gizmos.DrawRay(Position + offset4debug, pupetKin.LocalRotation * Vector3.forward * 0.15f);
                Gizmos.color = Color.yellow;
                Gizmos.DrawRay(Position + offset4debug, pupetKin.LocalRotation * Vector3.up * 0.15f);
                Gizmos.color = Color.grey;
                Gizmos.DrawRay(Position + offset4debug, pupetKin.LocalRotation * Vector3.right * 0.15f);

            }

        }



        public void DrawNormalTangent()
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(GetIKinematic().CenterOfMass, 0.01f);
          


            (var normal, var tangent) = GetIKinematic().TransformMatrix.ToNormalTangent();

            Gizmos.color = Color.grey;
            Gizmos.DrawRay(Position + 0.01f * (Vector3.up + Vector3.forward), (normal * 0.2f));
            Gizmos.color = Color.black;
            Gizmos.DrawRay(Position + 0.01f * (Vector3.up + Vector3.forward), (tangent * 0.2f));

            MjBody pupeteeredJoint4Debug = GetComponent<MjBody>();

            if (pupeteeredJoint4Debug != null)
            {

                IKinematic pupetKin = pupeteeredJoint4Debug.transform.GetIKinematic();

                //var parent = pupeteeredJoint4Debug.transform.parent.GetComponent<MjBody>();
                (var pupet_normal, var pupet_tangent) = pupetKin.TransformMatrix.ToNormalTangent();

                Gizmos.color = Color.cyan;
                Gizmos.DrawRay(Position, pupet_normal * 0.2f);
                Gizmos.color = Color.blue;
                Gizmos.DrawRay(Position, pupet_tangent * 0.2f);




            }
            //      (var normal, var tangent) = k.TransformMatrix.ToNormalTangent();
        }






        public class FiniteDifferenceBodyKinematics : IKinematic
        {
            MjFiniteDifferenceBody component;
            float mass;
            Vector3 inertiaLocalPos;
            Matrix4x4 inertiaRelMatrix;
            IKinematic parentKinematics;
            bool isRoot;

            public FiniteDifferenceBodyKinematics(MjFiniteDifferenceBody component) 
            { 
                this.component = component;


                var parent = this.component.transform.parent.GetComponentInParent<MjFiniteDifferenceBody>();  //Need to call at parent as GetComponentInParent is inclusive of the start transform.

                isRoot = !parent;
                if (parent) parentKinematics = parent.GetIKinematic();  //This doesn't create an endless loop as there's an exit condition.

                MjState.ExecuteAfterMjStart(MjInitialize);
            }

            private void MjInitialize()
            {
                mass = component.pairedBody.GetMass();
                inertiaLocalPos = component.pairedBody.GetLocalCenterOfMass();
                inertiaRelMatrix = component.pairedBody.GetInertiaToBodyMatrix();
            }
            public Vector3 Velocity => component.Velocity;

            public Vector3 AngularVelocity => component.AngularVelocity;

            public Vector3 LocalAngularVelocity => component.LocalAngularVelocity;

            public float Mass => mass;

         
            public Vector3 CenterOfMass => Matrix4x4.TRS(Position,
                                                 Rotation,
                                                 Vector3.one).MultiplyPoint3x4(inertiaLocalPos);
            

            
            public Matrix4x4 TransformMatrix => Matrix4x4.TRS(Position,
                                                              Rotation,
                                                              Vector3.one) * inertiaRelMatrix;
            
            public int index => throw new NotImplementedException();
            public Quaternion Rotation => component.Rotation;

            public Quaternion LocalRotation => component.LocalRotation;

            public Vector3 Position => component.Position;

            public Vector3 LocalVelocity => isRoot? Velocity : Velocity - parentKinematics.Velocity;

        

            public string Name => component.name;

            public GameObject gameObject => component.gameObject;

            public Vector3 Forward => Rotation * component.localForward;

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
        }
    }

    public interface IKinematicProvider
    {
        public IKinematic GetIKinematic();
    }

    public interface IFiniteDifferenceComponent
    {
        public void Step();
    }
}