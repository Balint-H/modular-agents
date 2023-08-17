using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PauseEnvironmentSteps : TrainingEventHandler
{

    [SerializeField]
    int framesToPause;

    [SerializeField]
    PausableEnvironmentStepHandler stepper;

    public override EventHandler Handler => (object sender, EventArgs e) => stepper.framesToWait=framesToPause;

}
