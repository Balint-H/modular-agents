using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

namespace ModularAgents.TrainingEvents
{
    public class RewardBelowThresholdEvent : TrainingEvent
    {
        [SerializeField, Tooltip("Deprecated, this event no longer requires reference to an Agent if \"signal\" is provided.")]
        Agent trackedAgent;

        [SerializeField]
        float rewardThreshold;

        [SerializeField]
        RewardSignal signal;

        private void Awake()
        {
            IEventsAgent eventsAgent = trackedAgent as IEventsAgent; //IEventsAgent is planned to be removed.
            
            if (eventsAgent != null)
            {
                eventsAgent.OnPostAction += ThresholdWrapper;
                return;
            }

            if(signal)
            {
                signal.OnCalculateReward += ThresholdWrapper;
            }
        }

        private void ThresholdWrapper(object sender, AgentEventArgs eventArgs)
        {
            if (eventArgs.reward >= rewardThreshold) return;

            OnTrainingEvent(eventArgs);
        }

        private void ThresholdWrapper(float reward)
        {
            if (reward >= rewardThreshold) return;

            OnTrainingEvent(EventArgs.Empty);
        }

    }
}