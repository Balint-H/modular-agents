using System.Collections.Generic;
using UnityEngine;
//using Mujoco;
//using Mujoco.Extensions;
using System.Linq;
using ModularAgents.DeepMimic;
//using ModularAgents.Kinematic.Mujoco;

namespace ModularAgentsRecorder
{
   
    public class FDSampler : MonoBehaviour
    {
      

        public enum DataToSave
        {
            qpos,
            qvel,
            all,
            counter
        }

        Animator animator = null;

        public DataToSave dataToSave = DataToSave.qpos;


        [SerializeField]
        MjFiniteDifferenceManager FDManager;


        /*
        [SerializeField]
        MjBody PupetRoot;
        */

        [SerializeField]
        ValueRecorder recorder;

        [SerializeField]
        MjDeepMimicObservations[] DMObs;

        [SerializeField]
        MjDeepMimicRewards[] DMRewards;



        List<MjFiniteDifferenceJoint> jointsFD = null;
        //List<MjBaseJoint> jointsPupet = null;


        private void OnEnable()
        {
            animator = FDManager.GetComponent<Animator>();
            GetJointOrderFD();
        }

        private void FixedUpdate()
        {


            CollectAll();


        }


        int counti = 0;
        void CollectCounter()
        {
            recorder.Record(new double[] { counti }, "counter_" );
            recorder.Record(new double[] { Time.fixedTimeAsDouble }, "time");
            
            counti++;
        }

        void CollectPhase()
        {
            if(animator)
                recorder.Record(new double[] { animator.GetCurrentAnimatorStateInfo(0).normalizedTime % 1 }, "phase");




        }






    void CollectAll()
        {
          
            CollectCounter();
            CollectPhase();

            CollectPositionsFD();

            CollectVelocitiesFD();

            //CollectPositionsPupet();
            //CollectVelocitiesPupet();
            foreach (MjDeepMimicObservations mjobs in DMObs)
            {
                mjobs.LogObservations(recorder);

                mjobs.LogObservationsSimpler(recorder);

            }

            foreach (MjDeepMimicRewards mjobs in DMRewards)
            {
                mjobs.LogRewards(recorder);

            }


        }



        //unsafe void CollectPositionsFD(object sender, EventArgs e)
        void CollectPositionsFD()
        {

          
            foreach(MjFiniteDifferenceJoint j in jointsFD) 
            {
                recorder.Record(j.QPos,"FD_QPos_" + j.name);

            }


        }


        //unsafe void CollectPositionsPupet(object sender, EventArgs e)

        /*
        void CollectPositionsPupet()
        {


            foreach (MjBaseJoint j in jointsPupet)
            {

                recorder.Record(j.GetQPos(), "Pupet_QPos_" + j.name);
               // recorder.Record(j.QPos, "FD_" + j.name);

            }


        }*/


        // unsafe void CollectVelocitiesFD(object sender, EventArgs e)
        void CollectVelocitiesFD()
        {


            foreach (MjFiniteDifferenceJoint j in jointsFD)
            {
                recorder.Record(j.QVel, "FD_QVel_" +  j.name);

            }


        }

        //unsafe void CollectVelocitiesPupet(object sender, EventArgs e)
        /*
        void CollectVelocitiesPupet()
        {


            foreach (MjBaseJoint j in jointsPupet)
            {
                recorder.Record(j.GetQVel(), "Pupet_QVel_" + j.name);

            }


        }
        */


        void GetJointOrderFD()
        {
            
            MjFiniteDifferenceJoint[] fdJoints = FDManager.transform.GetComponentsInChildren<MjFiniteDifferenceJoint>();

            jointsFD =fdJoints.OrderBy(x => x.PairedJoint.MujocoId).ToList();

        }

        /*
        void GetJointOrderPupet()
        {

            MjBaseJoint[] fdJoints = PupetRoot.transform.GetComponentsInChildren<MjBaseJoint>();

            jointsPupet = fdJoints.OrderBy(x => x.MujocoId).ToList();

        }*/

    }


}