using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

namespace ModularAgents.TrainingEvents
{ 
    /// <summary>
    /// Triggered at the start of a ModularAgent's episode
    /// </summary>
    public class EpisodeBeginEvent : TrainingEvent
    {
        [SerializeField]
        Agent trackedAgent;
        private void Awake()
        {
            switch (trackedAgent)
            {
                case IEventsAgent eventsAgent: //IEventsAgent is deprecated, but supported for now
                    eventsAgent.OnBegin += BeginWrapper;
                    break;
                case ModularAgent modularAgent:
                    modularAgent.OnBegin += BeginWrapper;
                    break;
                default:
                    throw new InvalidCastException("Agent should implement IEventsAgent or be a ModularAgent");
            }
        
        }

        private void BeginWrapper(object sender, AgentEventArgs eventArgs)
        {
            OnTrainingEvent(eventArgs);
        }

        private void BeginWrapper(object sender, EventArgs eventArgs)
        {
            OnTrainingEvent(eventArgs);
        }
    }
}