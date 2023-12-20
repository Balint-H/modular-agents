using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using Mujoco.Extensions;
using System.Linq;

namespace ModularAgentsRecorder
{ 
    public class FDSampler : TrainingEventHandler
    {

        public enum DataToSave
        {
            qpos,
            qvel
        }


        public DataToSave dataToSave = DataToSave.qpos;


        public override EventHandler Handler => CollectData;

        [SerializeField]
        MjFiniteDifferenceManager FDManager;

        [SerializeField]
        ValueRecorder recorder;


        List<MjFiniteDifferenceJoint> joints = null;


        void CollectData(object sender, EventArgs e)
        {

            if (joints == null)
            {
                GetJointOrder();

            }
            switch (dataToSave)
            {
                case DataToSave.qvel:
                    CollectVelocities(sender, e);
                    break;

                case DataToSave.qpos:
                    CollectPositions(sender, e);
                    break;


            }



        }


        unsafe void CollectPositions(object sender, EventArgs e)
        {

          
            foreach(MjFiniteDifferenceJoint j in joints) 
            {
                recorder.Record(j.QPos, j.name);

            }


        }


        unsafe void CollectVelocities(object sender, EventArgs e)
        {


            foreach (MjFiniteDifferenceJoint j in joints)
            {
                recorder.Record(j.QVel, j.name);

            }


        }


        void GetJointOrder()
        {
            
            MjFiniteDifferenceJoint[] fdJoints = FDManager.transform.GetComponentsInChildren<MjFiniteDifferenceJoint>();

            joints =fdJoints.OrderBy(x => x.PairedJoint.MujocoId).ToList();

        }



    }


}