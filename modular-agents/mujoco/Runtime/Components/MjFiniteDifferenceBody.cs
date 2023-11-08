using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using ModularAgents.MotorControl.CircularBuffer;
using System.Linq;
using Mujoco.Extensions;

namespace ModularAgents.Kinematic.Mujoco
{
    /// <summary>
    /// A MocapBody that can provide 6 DOF velocity estimates based on finite differences.
    /// </summary>

    public class MjFiniteDifferenceBody : MonoBehaviour, IKinematicProvider, IFiniteDifferenceComponent
    {
        [SerializeField]
        MjBody pairedBody;

        [SerializeField]
        MjBody pupeteeredJoint4Debug;


        public MjBody PupeteeredBody {  set => pupeteeredJoint4Debug = value; }

        public MjBody PairedBody { get => pairedBody; set => pairedBody = value; }
        IKinematic pairedKinematics = null;


        Vector3 prevPosition;
        Quaternion prevRotation;


        Quaternion prevLocalRotation  = Quaternion.identity;

        Quaternion prevPupeteeredLocalRotation = Quaternion.identity;
        Quaternion pupeteeredLocalRotation = Quaternion.identity;

        private Vector3 Position => transform.position;
        private Quaternion Rotation => transform.rotation;
        private Quaternion LocalRotation => transform.localRotation;




        private float fs;

        private Vector3 Velocity => (Position - prevPosition)*fs;
        //private Vector3 AngularVelocity => Utils.QuaternionError(Rotation, prevRotation)*fs;
        private Vector3 AngularVelocity => Utils.QuaternionError2(Rotation, prevRotation) * fs;






        //  public Vector3 LocalAngularVelocity => isRoot ? AngularVelocity : AngularVelocity - parentKinematics.AngularVelocity;

        // private Vector3 LocalAngularVelocity =>   Utils.QuaternionError2(transform.localRotation, prevLocalRotation) * fs;
        private Vector3 LocalAngularVelocity => QuaternionLocalVel();



        Vector3 QuaternionLocalVelTest1()
        {

            //from here: https://mariogc.com/post/angular-velocity-quaternions/
            return new Vector3(
             2 * fs * (prevLocalRotation.w * LocalRotation.x - prevLocalRotation.x * LocalRotation.w - prevLocalRotation.y * LocalRotation.z + prevLocalRotation.z * LocalRotation.y ),
             2 * fs * (prevLocalRotation.w * LocalRotation.y + prevLocalRotation.x * LocalRotation.z - prevLocalRotation.y * LocalRotation.w - prevLocalRotation.z * LocalRotation.x ),
             2 * fs * (prevLocalRotation.w * LocalRotation.z - prevLocalRotation.x * LocalRotation.y + prevLocalRotation.y * LocalRotation.x - prevLocalRotation.z * LocalRotation.w )
            );

        }

        //https://github.com/google-deepmind/mujoco/blob/deb14dc081c956997d2398cff367168219ce9939/src/engine/engine_util_spatial.c#L236

        /*
        
        // Subtract quaternions, express as 3D velocity: qb*quat(res) = qa.

void mju_subQuat(mjtNum res[3], const mjtNum qa[4], const mjtNum qb[4]) {
  // qdif = neg(qb)*qa
  mjtNum qneg[4], qdif[4];
  mju_negQuat(qneg, qb);
  mju_mulQuat(qdif, qneg, qa);

  // convert to 3D velocity
  mju_quat2Vel(res, qdif, 1);
}

        
// convert quaternion (corresponding to orientation difference) to 3D velocity
void mju_quat2Vel(mjtNum res[3], const mjtNum quat[4], mjtNum dt) {
  mjtNum axis[3] = {quat[1], quat[2], quat[3]};
  mjtNum sin_a_2 = mju_normalize3(axis);
  mjtNum speed = 2 * mju_atan2(sin_a_2, quat[0]);

  // when axis-angle is larger than pi, rotation is in the opposite direction
  if (speed>mjPI) {
    speed -= 2*mjPI;
  }
  speed /= dt;

  mju_scl3(res, axis, speed);
}

         */

        Vector3 QuaternionLocalVel()
        {
            //Quaternion negLocRot = new Quaternion( - LocalRotation.x, -LocalRotation.y, -LocalRotation.z, -LocalRotation.w);

            //Quaternion qdif = Quaternion.Inverse(LocalRotation) * prevLocalRotation;
            //Quaternion qdif = LocalRotation * Quaternion.Inverse(prevLocalRotation);
            Quaternion qdif = Quaternion.Inverse(prevLocalRotation) * LocalRotation;

            Vector3 axis = new Vector3(qdif.x, qdif.y, qdif.z);

            float speed = 2 * Mathf.Atan2(axis.magnitude, qdif.w) * fs;

            return speed * axis.normalized;
            //return  axis;




        }





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
            pairedKinematics = pairedBody.transform.GetIKinematic();



        }


        public void FixedUpdate()
        {
            Step();
        }


        public void Step()
        {
            prevPosition = transform.position;
            prevRotation = transform.rotation;

            prevLocalRotation = transform.localRotation;


           // prevPupeteeredLocalRotation = pupeteeredLocalRotation;
          //  pupeteeredLocalRotation = pupeteeredJoint4Debug.transform.GetIKinematic().LocalRotation ;

    

    }

    public IKinematic GetIKinematic()
        {
            if(kinematics == null) kinematics = new FiniteDifferenceBodyKinematics(this);
            return kinematics;
        }

        private void OnDrawGizmosSelected()
        {
            if (!Application.isPlaying) return;
            //((MocapBodyKinematics)GetIKinematic()).Draw();

           // Draw2();

            Draw();
        }

        public void Draw()
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(GetIKinematic().CenterOfMass, 0.01f);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(Position, Rotation * Vector3.forward * 0.015f);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(Position, Rotation * Vector3.up * 0.015f);
            Gizmos.color = Color.red;
            Gizmos.DrawRay(Position, Rotation * Vector3.right * 0.015f);
           // Gizmos.color = Color.cyan;
           // Gizmos.DrawRay(Position, Velocity * 0.05f);


            //if(pairedKinematics != null) 
            {

                Gizmos.color = Color.cyan;
                //Gizmos.DrawRay(pairedKinematics.Position, Velocity * 0.05f);
                //Gizmos.DrawRay(Position, LocalAngularVelocity * 0.2f);

                //Gizmos.DrawRay(Position + offset4debug, MjEngineTool.UnityVector3(AngularVelocity) * 0.2f);
                //Gizmos.DrawRay(Position + offset4debug, AngularVelocity * 0.2f);

              
                

               // Gizmos.DrawRay(Position, MjEngineTool.UnityVector3( LocalAngularVelocity) * 0.2f);

                //Gizmos.DrawRay(pairedKinematics.Position - offset4debug, pairedKinematics.Velocity * 0.05f);
                //Gizmos.DrawRay(Position - offset4debug, pairedKinematics.AngularVelocity * 0.2f);
                //Gizmos.DrawRay(Position, pairedKinematics.Velocity * 0.05f);

                Gizmos.color = Color.grey;

                Gizmos.DrawRay(Position , LocalAngularVelocity * 0.2f);



                if (pupeteeredJoint4Debug != null)
                {
                    Gizmos.color = Color.blue;
                    IKinematic pupetKin = pupeteeredJoint4Debug.transform.GetIKinematic();
                    //Gizmos.DrawRay(pairedKinematics.Position + offset4debug, pupetKin.Velocity * 0.05f);
                    //Gizmos.DrawRay(Position + offset4debug, pupetKin.AngularVelocity * 0.2f);
                    Gizmos.DrawRay(Position, pupetKin.LocalAngularVelocity * 0.2f);


                    //MjEngineTool.
                    //lets calculate an angular velocity using a similar method, but using mujoco:

                    //        public Quaternion QLocalRotation => isRoot ? Rotation : Quaternion.Inverse(MjState.GlobalRotation(parentBody)) * Rotation;




                }


            }

        }


        public void Draw2()
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(GetIKinematic().CenterOfMass, 0.01f);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(Position, LocalRotation * Vector3.forward * 0.15f);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(Position, LocalRotation * Vector3.up * 0.15f);
            Gizmos.color = Color.red;
            Gizmos.DrawRay(Position, LocalRotation * Vector3.right * 0.15f);
     

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






        private class FiniteDifferenceBodyKinematics : IKinematic
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


                //var parent = this.component.GetComponentInParent<MjFiniteDifferenceBody>(); //this gives itself, not necessarily its parent

               var parentTransform = this.component.transform.parent;

                var parent = parentTransform.GetComponent<MjFiniteDifferenceBody>();

                isRoot = !parent;

                //if (parent) parentKinematics = parent.GetIKinematic(); //this creates an endless loop

                if (!isRoot)
                {
                    if (parent.kinematics == null)
                        Debug.Log("the kinematics of " + parent.name + " have not been set, this is a problem for: " + this.Name);
                    else
                        parentKinematics = parent.kinematics;                    //this should slove the previous loop problem

                }
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

            public float Mass => mass;

            public Vector3 CenterOfMass => Matrix4x4.TRS(Position,
                                                 Rotation,
                                                 Vector3.one).MultiplyPoint3x4(inertiaLocalPos);

            public Matrix4x4 TransformMatrix => Matrix4x4.TRS(Position,
                                                              Rotation,
                                                              Vector3.one) * inertiaRelMatrix;

            public int index => -component.pairedBody.MujocoId;

            public Quaternion Rotation => component.Rotation;

            public Quaternion LocalRotation => isRoot? Rotation : Quaternion.Inverse(parentKinematics.Rotation) * Rotation;

            public Vector3 Position => component.Position;

            public Vector3 LocalVelocity => isRoot? Velocity : Velocity - parentKinematics.Velocity;

            public Vector3 LocalAngularVelocity => isRoot ? AngularVelocity : AngularVelocity - parentKinematics.AngularVelocity;

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