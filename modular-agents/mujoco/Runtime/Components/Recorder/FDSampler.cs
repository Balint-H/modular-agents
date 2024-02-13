using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using Mujoco.Extensions;
using System.Linq;
using ModularAgents.DeepMimic;
using UnityEditor;
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

        public bool recordOnlyOneAnimCycle = false;
        public bool useMujocoNames = false;

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
          
        }

        private void Start()
        {

            MjState.ExecuteAfterMjStart(GetJointOrderFD);
            //GetJointOrderFD();
        }

        float lastPhaseRecorded = 0;

        private void FixedUpdate()
        {


            CollectData();
            if (recordOnlyOneAnimCycle)
            {
                if (!animator)
                    Debug.LogError("no animator to check");
                float currentPhase =  animator.GetCurrentAnimatorStateInfo(0).normalizedTime % 1;
                if (currentPhase > lastPhaseRecorded)
                    lastPhaseRecorded = currentPhase;
                else
#if UNITY_EDITOR
                    EditorApplication.ExitPlaymode();
#else
                    Application.Quit();
#endif


            }

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






    void CollectData()
        {

            switch (dataToSave)
            {
                case (DataToSave.qpos):
                    recorder.Record(new double[] { Time.fixedTimeAsDouble }, "time");
                    CollectPositionsFD();
                    break;

                case (DataToSave.qvel):
                    recorder.Record(new double[] { Time.fixedTimeAsDouble }, "time");
                    CollectVelocitiesFD();
                    break;


                case (DataToSave.all):
                    
                   


          
                    CollectCounter();
                    CollectPhase();

                    CollectPositionsFD("pos_");

                    CollectVelocitiesFD("vel_");

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
                    break;



            }


        }



        //unsafe void CollectPositionsFD(object sender, EventArgs e)
        void CollectPositionsFD(string prefix = "")
        {

          
            foreach(MjFiniteDifferenceJoint j in jointsFD) 
            {


                if(useMujocoNames)
                    recorder.Record(j.QPos, j.PairedJoint.MujocoName);
                else
                    recorder.Record(j.QPos, prefix + j.name);

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
        void CollectVelocitiesFD(string prefix = "")
        {


            foreach (MjFiniteDifferenceJoint j in jointsFD)
            {
                if (useMujocoNames)
                    recorder.Record(j.QVel, j.PairedJoint.MujocoName );
                else
                    recorder.Record(j.QVel, prefix +  j.name );

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

            //foreach(MjFiniteDifferenceJoint mjfd in jointsFD)
            //    Debug.Log("joints: " + mjfd.name + " mujocName: " + mjfd.PairedJoint.MujocoName + " " + mjfd.PairedJoint.MujocoId + " name: " + mjfd.PairedJoint.transform.name);

        }

        /*
        void GetJointOrderPupet()
        {

            MjBaseJoint[] fdJoints = PupetRoot.transform.GetComponentsInChildren<MjBaseJoint>();

            jointsPupet = fdJoints.OrderBy(x => x.MujocoId).ToList();

        }*/

    }


}