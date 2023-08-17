using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

public class AgentActionEvent : TrainingEvent
{

    [SerializeField]
    Agent trackedAgent;

    private void Awake()
    {
        IEventsAgent eventsAgent = trackedAgent as IEventsAgent;
        if (eventsAgent == null)
        {
            throw new InvalidCastException("Agent should implement IEventsAgent");
        }

        eventsAgent.OnPostAction += ActionWrapper;
    }

    private void ActionWrapper(object sender, AgentEventArgs eventArgs)
    {
        OnTrainingEvent(eventArgs);
    }

}
