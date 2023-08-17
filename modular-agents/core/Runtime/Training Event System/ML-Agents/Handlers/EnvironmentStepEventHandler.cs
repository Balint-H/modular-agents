using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class EnvironmentStepEventHandler : TrainingEventHandler
{
    public override EventHandler Handler => (object sender, EventArgs e) => Academy.Instance.EnvironmentStep() ;

    private void Awake()
    {
        Academy.Instance.AutomaticSteppingEnabled = false;
    }


}
