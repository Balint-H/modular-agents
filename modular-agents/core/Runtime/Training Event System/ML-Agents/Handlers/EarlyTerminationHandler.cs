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

    [SerializeField, Tooltip("Determines wheter EndEpisode() or InterruptEpisode() is called. Set false if the episode needs to end from no fault of the agent.")]
    bool agentsFault = true;

    public override EventHandler Handler => Terminate;

    private void Terminate(object sender, EventArgs args)
    {
        if(agentsFault)
        {
            agent.EndEpisode();
        }
        else
        {
            agent.EpisodeInterrupted();
        }
    }
}
