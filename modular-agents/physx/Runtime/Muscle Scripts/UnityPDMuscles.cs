using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;
using System.Linq;


namespace ModularAgents.MotorControl.PhysX
{

    public class UnityPDMuscles : UnityMuscles
    {

        //this functions does essentially the samethan the old ArticulationMuscles in package marathon.muscles, but WITHOUT delaing with normalized Rotations



        [SerializeField]
        protected Transform RagdollRoot;

        //Rigidbody[] trackedRigidBodies;




        [SerializeField]
        private float stiffness=100;
        [SerializeField]
        private float damping = 10;


        // Start is called before the first frame update
        void Start()
        {
           // InitMotors();   //not needed, already done in parent class, within method OnAgentInitialize()
        }

        // Update is called once per frame
        void Update()
        {

        }

        public virtual IEnumerable<ArticulationBody> Joints { get => RagdollRoot.GetComponentsInChildren<ArticulationBody>(); }

        // public override int ActionSpaceSize => Joints.DofSum();



        /*
        public void MimicRigidBodies(List<Rigidbody> targets, float deltaTime)
        {
           

            int im = 0;
            foreach (var a in targets)
            {
                targetRotations[im] = Mathf.Deg2Rad * Utils.GetSwingTwist(a.transform.localRotation);
                classicPD(_motors[im], targetRotations[im], deltaTime);
                im++;
            }

        }
        */





        public override void ApplyActions(float[] actions, float actionTimeDelta)
     
        {
            int i = 0;

            float[] stateactions = GetActionsFromState();


            foreach (var m in _motors)
            {
                if (m.isRoot)
                    continue;
                //Debug.Log("motor number " + im + " action number: " + i);
                Vector3 targetRotation = Vector3.zero;
                if (m.jointType != ArticulationJointType.SphericalJoint)
                    continue;
                if (m.twistLock != ArticulationDofLock.LockedMotion) { 
                    targetRotation.x = stateactions[i] + actions[i];
                    i++;
                }
                if (m.swingYLock != ArticulationDofLock.LockedMotion)
                {
                    targetRotation.y = stateactions[i] + actions[i];
                    i++;
                }
                if (m.swingZLock != ArticulationDofLock.LockedMotion)
                {
                    targetRotation.z = stateactions[i] + actions[i];
                    i++;
                }

              
                classicPD(m, (float3) targetRotation, actionTimeDelta);

            }

        }



        void classicPD(ArticulationBody joint, double3 targetRotation, float actionTimeDelta)
        {

          


            double3 targetVel = GetTargetVelocity(joint, targetRotation, actionTimeDelta);



            if (joint.twistLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.xDrive;
                drive.target = Mathf.Rad2Deg *(float)targetRotation.x;
                drive.targetVelocity = Mathf.Rad2Deg *  (float)targetVel.x;
                drive.stiffness = stiffness;
                drive.damping = damping;

                joint.xDrive = drive;
            }

            if (joint.swingYLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.yDrive;
                drive.target = Mathf.Rad2Deg * (float)targetRotation.y;
                drive.targetVelocity = Mathf.Rad2Deg * (float)targetVel.y;
                drive.stiffness = stiffness;
                drive.damping = damping;

                joint.yDrive = drive;
            }

            if (joint.swingZLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.zDrive;
                drive.target = Mathf.Rad2Deg * (float)targetRotation.z;
                drive.targetVelocity = Mathf.Rad2Deg * (float)targetVel.z;
                drive.stiffness = stiffness;
                drive.damping = damping;

                joint.zDrive = drive;
            }



        }


        private static double3 GetTargetVelocity(ArticulationBody joint, double3 targetRotation, float timeDelta)
        {


            double3 targetVelocity = UtilsMathematics.AngularVelocityInReducedCoordinates(UtilsMathematics.GetSwingTwist(joint.transform.localRotation), targetRotation, timeDelta);


            targetVelocity = UtilsMathematics.Clamp(targetVelocity, (double)joint.maxAngularVelocity);


            return targetVelocity;



        }
    }

}
