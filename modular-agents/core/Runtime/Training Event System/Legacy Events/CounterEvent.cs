using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CounterEvent : TrainingEventHandler
{
    // Start is called before the first frame update
    [SerializeField]
    int maxCount;

    int runningCount;

    [SerializeField]
    TrainingEventHandler wrappedHandler;

    public override EventHandler Handler => (object sender, System.EventArgs args) => RunningCount++; 

    public int RunningCount
    {
        get => runningCount;

        set
        {
            if (value > maxCount)
            {
                wrappedHandler.Handler?.Invoke(this, System.EventArgs.Empty);
                runningCount = 0;
                
            }
            else
            {
                runningCount = value;
            }
        }
    }

}
