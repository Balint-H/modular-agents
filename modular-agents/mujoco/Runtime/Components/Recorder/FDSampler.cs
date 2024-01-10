using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using Mujoco.Extensions;
using System.Linq;
using ModularAgents.DeepMimic;
using ModularAgents.Kinematic.Mujoco;

namespace ModularAgentsRecorder
{ 
    public class FDSampler : TrainingEventHandler
    {
        [Range(1, 10)]
        public int LoggingPeriod = 2;
        int loggingCounter = 0;

        public enum DataToSave
        {
            qpos,
            qvel,
            all
        }


        public DataToSave dataToSave = DataToSave.qpos;


        public override EventHandler Handler => CollectDataFD;

        [SerializeField]
        MjFiniteDifferenceManager FDManager;

        [SerializeField]
        MjBody PupetRoot;


        [SerializeField]
        ValueRecorder recorder;

        [SerializeField]
        MjDeepMimicObservations[] DMObs;


        List<MjFiniteDifferenceJoint> jointsFD = null;
        List<MjBaseJoint> jointsPupet = null;
        


        void CollectDataFD(object sender, EventArgs e)
        {
            loggingCounter++;
            if(loggingCounter % LoggingPeriod == 0) 
            {
          

                if (jointsFD == null)
                {
                    GetJointOrderFD();

                }


                if (jointsPupet == null)
                {
                    GetJointOrderPupet();

                }



                switch (dataToSave)
                {
                    case DataToSave.qvel:
                        CollectVelocitiesFD(sender, e);
                        break;

                    case DataToSave.qpos:
                        CollectPositionsFD(sender, e);
                        break;

                    case DataToSave.all:
                        CollectAll(sender, e);
                        break;


                }


            }


        }

        unsafe void CollectAll(object sender, EventArgs e)
        {
            CollectPositionsFD(sender, e);

            CollectVelocitiesFD(sender, e);

            CollectPositionsPupet(sender, e);
            CollectVelocitiesPupet(sender, e);
            foreach (MjDeepMimicObservations mjobs in DMObs)
            {
                mjobs.LogObservations(recorder);

                mjobs.LogObservationsSimpler(recorder);

            }

        }



        unsafe void CollectPositionsFD(object sender, EventArgs e)
        {

          
            foreach(MjFiniteDifferenceJoint j in jointsFD) 
            {
                recorder.Record(j.QPos,"FD_QPos_" + j.name);

            }


        }


        unsafe void CollectPositionsPupet(object sender, EventArgs e)
        {


            foreach (MjBaseJoint j in jointsPupet)
            {

                recorder.Record(j.GetQPos(), "Pupet_QPos_" + j.name);
               // recorder.Record(j.QPos, "FD_" + j.name);

            }


        }


        unsafe void CollectVelocitiesFD(object sender, EventArgs e)
        {


            foreach (MjFiniteDifferenceJoint j in jointsFD)
            {
                recorder.Record(j.QVel, "FD_QVel_" +  j.name);

            }


        }

        unsafe void CollectVelocitiesPupet(object sender, EventArgs e)
        {


            foreach (MjBaseJoint j in jointsPupet)
            {
                recorder.Record(j.GetQVel(), "Pupet_QVel_" + j.name);

            }


        }



        void GetJointOrderFD()
        {
            
            MjFiniteDifferenceJoint[] fdJoints = FDManager.transform.GetComponentsInChildren<MjFiniteDifferenceJoint>();

            jointsFD =fdJoints.OrderBy(x => x.PairedJoint.MujocoId).ToList();

        }

        void GetJointOrderPupet()
        {

            MjBaseJoint[] fdJoints = PupetRoot.transform.GetComponentsInChildren<MjBaseJoint>();

            jointsPupet = fdJoints.OrderBy(x => x.MujocoId).ToList();

        }

    }


}