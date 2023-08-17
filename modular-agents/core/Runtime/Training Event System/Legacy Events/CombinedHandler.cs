using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

public class CombinedHandler : TrainingEventHandler
{
    [SerializeField]
    List<TrainingEventHandler> handlers;

    public override EventHandler Handler => TriggerAll;

    private void TriggerAll(object sender, EventArgs args)
    {

        foreach (var handle in handlers)
        {
            handle.Handler(sender, args);
        }
    }


}