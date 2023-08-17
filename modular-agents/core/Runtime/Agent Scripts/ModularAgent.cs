using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.Assertions;

using System;
using Unity.MLAgents.Policies;

namespace ModularAgents
{
    /// <summary>
    /// An application agnostic agent that exposes events for other scripts to hook into, to determine the environment update loop.
    /// </summary>
    [DefaultExecutionOrder(500)]
    [AddComponentMenu("Scripts/Modular Agent", -1)]
    public class ModularAgent : Agent
    {
        [SerializeField]
        ActuatorSignal actuatorSignal;

        [SerializeField]
        ObservationSignal observationSignal;

        [SerializeField]
        List<RewardSignal> rewardSignals;

        [SerializeField, Tooltip("Will be passed to BehaviorParameters, which hides this public field in the editor.")]
        public bool useChildActuators = true;

        public event EventHandler OnBeforeAction
        {
            add
            {
                actuatorSignal.OnBeforeAction += value;
            }
            remove
            {
                actuatorSignal.OnBeforeAction -= value;
            }
        }

        public event EventHandler OnAfterAction
        {
            add
            {
                actuatorSignal.OnAfterAction += value;
            }
            remove
            {
                actuatorSignal.OnAfterAction -= value;
            }
        }

        public event EventHandler OnBegin;
        public event EventHandler OnStart;

        public int ObservationSpaceSize => observationSignal.Size;

        private void Start()
        {
            if(observationSignal) observationSignal.OnAgentStart();
            foreach (RewardSignal rewardSignal in rewardSignals)
            {
                if(rewardSignal) rewardSignal.OnAgentStart(this);
            }
            OnStart?.Invoke(this, AgentEventArgs.Empty);

        }

        protected override void Awake()
        {
            GetComponent<BehaviorParameters>().UseChildActuators = useChildActuators;
            base.Awake();
        }

        public override void OnEpisodeBegin()
        {
            OnBegin?.Invoke(this, AgentEventArgs.Empty);
        }

        private void Reset()
        {
            if (!observationSignal)
            {
                var obs = new GameObject("Observation Signal");
                obs.transform.parent = transform;
                observationSignal = obs.AddComponent<ObservationSignal>();

            }

            if(rewardSignals == null || rewardSignals.Count == 0)
            {
                rewardSignals = new List<RewardSignal>();
                var rew = new GameObject("Reward Signal");
                rew.transform.parent = transform;
                rewardSignals.Add(rew.AddComponent<RewardSignal>());
            }

            if (!actuatorSignal)
            {
                var act = new GameObject("Actuator Signal");
                act.transform.parent = transform;
                actuatorSignal = act.AddComponent<ActuatorSignal>();
            }

        }

    }


}