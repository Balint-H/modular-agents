using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;
using System.Linq;

public class EarlyTerminationHandler : TrainingEventHandler
{
    [SerializeField]
    Agent agent;

    public override EventHandler Handler => Terminate;

    private void Terminate(object sender, EventArgs args)
    {
        agent.EndEpisode();
    }
}
