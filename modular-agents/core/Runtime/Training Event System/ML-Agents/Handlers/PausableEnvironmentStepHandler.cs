using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class PausableEnvironmentStepHandler : TrainingEventHandler
{
    public int framesToWait;

    public override EventHandler Handler => (object sender, EventArgs e) => 
    { 
        if (framesToWait <= 0) 
        { 
            Academy.Instance.EnvironmentStep();
            return;
        }
        framesToWait--;
    
    };

    private void Awake()
    {
        Academy.Instance.AutomaticSteppingEnabled = false;
    }


}
