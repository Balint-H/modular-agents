using System;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using Mujoco.Extensions;

namespace ModularAgentsRecorder
{
    public class QPosSampler : TrainingEventHandler
    {
        public override EventHandler Handler => CollectPositions;

        [SerializeField]
        List<MjBaseJoint> trackedDofs;

        [SerializeField]
        ValueRecorder recorder;



        unsafe void CollectPositions(object sender, EventArgs e)
        {
            if (!recorder) return;
            foreach (var j in trackedDofs)
            {
                recorder.Record(j.GetQPos(), j.name);
            }
        }
    }
}