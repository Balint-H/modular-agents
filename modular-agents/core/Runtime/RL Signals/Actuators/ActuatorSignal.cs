using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Actuators;
using UnityEngine;
using System.Linq;
using ModularAgents.MotorControl;

namespace ModularAgents
{

    /// <summary>
    /// The ModularActuator references other ActuatorComponents in the scene that may not necessarily be child gameobjects of the Agent. 
    /// </summary>
    public class ActuatorSignal : ActuatorComponent
    {
        [SerializeField]
        List<ActuatorComponent> actuatorComponents;

        public event EventHandler OnBeforeAction;
        public event EventHandler OnAfterAction;

        public override ActionSpec ActionSpec => ActionSpec.MakeContinuous(actuatorComponents.Sum(a => a.ActionSpec.NumContinuousActions));

        public override IActuator[] CreateActuators()
        {
            List<IActuator> actuators = new List<IActuator>();
            foreach (var actuatorComponent in actuatorComponents)
            {
                actuators.AddRange(actuatorComponent.CreateActuators());
            }

            if (actuators.Count >= 1)
            {
                actuators[0] = new EventActuator(actuators[0], this, fireBeforeEvent: true, fireAfterEvent: true);
            }
            else
            {
                actuators[0] = new EventActuator(actuators[0], this, fireBeforeEvent: true);
                actuators[actuators.Count - 1] = new EventActuator(actuators[actuators.Count - 1], this, fireAfterEvent: true);
            }
            return actuators.ToArray();
        }


        private class EventActuator : ActuatorWrapper
        {
            protected override string WrappingName => "Event";
            ActuatorSignal eventComponent;
            bool fireBeforeEvent;
            bool fireAfterEvent;

            public EventActuator(IActuator wrappedActuator, ActuatorSignal eventComponent, bool fireBeforeEvent = false, bool fireAfterEvent = false)
            {
                this.wrappedActuator = wrappedActuator;
                this.eventComponent = eventComponent;
                this.fireBeforeEvent = fireBeforeEvent;
                this.fireAfterEvent = fireAfterEvent;
            }

            protected override void PostprocessActionBuffers(ActionBuffers actionBuffersOut)
            {
                if (fireAfterEvent)
                {
                    eventComponent.OnAfterAction?.Invoke(this, EventArgs.Empty);
                }
            }

            protected override ActionBuffers PreprocessActionBuffers(ActionBuffers actionBuffersOut)
            {
                if (fireBeforeEvent)
                {
                    eventComponent.OnBeforeAction?.Invoke(this, EventArgs.Empty);
                }
                return actionBuffersOut;
            }

            protected override void ResetWrapperData()
            {

            }
        }
    }
}