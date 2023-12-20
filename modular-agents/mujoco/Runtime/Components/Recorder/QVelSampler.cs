using Mujoco.Extensions;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Mujoco;

namespace GaitLab
{
    public class QVelSampler : TrainingEventHandler
    {
        public override EventHandler Handler => CollectVelocities;

        [SerializeField]
        List<MjBaseJoint> trackedDofs;

        [SerializeField]
        ValueRecorder recorder;



        unsafe void CollectVelocities(object sender, EventArgs e)
        {
            if (!recorder) return;
            foreach (var j in trackedDofs)
            {
                recorder.Record(j.GetQVel(), j.name);
            }
        }
    }
}