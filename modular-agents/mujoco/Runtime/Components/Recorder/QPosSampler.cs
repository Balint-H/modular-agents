using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Mujoco;
using Mujoco.Extensions;

namespace GaitLab
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