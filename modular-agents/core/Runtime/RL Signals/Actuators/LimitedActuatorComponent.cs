using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Actuators;
using UnityEngine;

namespace ModularAgents.MotorControl
{
    /// <summary>
    /// Limits all dimensions of the action vector to the range provided.
    /// </summary>
    public class LimitedActuatorComponent : ActuatorComponentWrapper
    {
        [SerializeField, Tooltip("Lower and upper limits each action component will be clipped by.")]
        Vector2 range;

        public override ActionSpec ActionSpec => wrappedActuatorComponent.ActionSpec;

        public override IActuator WrapActuator(IActuator actuator)
        {
            return new LimitedActuator(actuator, this);
        }

        private class LimitedActuator : ActuatorWrapper
        {

            LimitedActuatorComponent component;

            public LimitedActuator(IActuator actuator, LimitedActuatorComponent component)
            {
                this.component = component;
                this.wrappedActuator = actuator;
            }   

            protected override string WrappingName => "Limited";

            protected override void PostprocessActionBuffers(ActionBuffers actionBuffersOut)
            {
                

            }

            protected override ActionBuffers PreprocessActionBuffers(ActionBuffers actionBuffersOut)
            {
                var actions = actionBuffersOut.ContinuousActions;
                for (int i = 0; i < actions.Length; i++)
                {
                    actions[i] = Mathf.Clamp(actions[i], component.range.x, component.range.y);
                }
                return actionBuffersOut;
            }

            protected override void ResetWrapperData()
            {

            }
        }
    }
}