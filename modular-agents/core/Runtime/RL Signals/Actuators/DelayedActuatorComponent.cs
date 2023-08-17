using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents.Actuators;
using UnityEngine;

namespace ModularAgents.MotorControl
{ 
public class DelayedActuatorComponent : ActuatorComponentWrapper
{

    [SerializeField]
    int delay;

    public override ActionSpec ActionSpec => wrappedActuatorComponent.ActionSpec;

    public override IActuator WrapActuator(IActuator actuator)
    {
        return new DelayedActuator(actuator, this);
    }

    class DelayedActuator : ActuatorWrapper
    {
        private int delay;
        private DelayedActuatorComponent delayedActuatorComponent;

        public DelayedActuator(IActuator wrappedActuator, DelayedActuatorComponent delayedActuatorComponent)
        {
            this.wrappedActuator = wrappedActuator;
            this.delay = delayedActuatorComponent.delay;
            this.actionMemory = new CircularBuffer.CircularBuffer<ActionBuffers>(delay + 1);
            this.delayedActuatorComponent = delayedActuatorComponent;
        }

        private CircularBuffer.CircularBuffer<ActionBuffers> actionMemory;

        protected override string WrappingName => "Delayed";

        private void CheckMemorySize()
        {
            if (delayedActuatorComponent.delay != delay)
            {
                delay = delayedActuatorComponent.delay;
                var newMemory = new CircularBuffer.CircularBuffer<ActionBuffers>(delay + 1);
                for (int i = 0; i < actionMemory.Capacity-1; i++)
                {
                    newMemory.PushBack(actionMemory[i]);
                }
                actionMemory = newMemory;
            }
        }

        protected override void ResetWrapperData()
        {
            actionMemory.Clear();
        }

        protected override ActionBuffers PreprocessActionBuffers(ActionBuffers actionBuffersOut)
        {
            CheckMemorySize();
            //Debug.Log("old: " + string.Join(", ", actionMemory.Select(a => a.ContinuousActions.Array[0].ToString())));
            var continuousSegment = new ActionSegment<float>((float[])actionBuffersOut.ContinuousActions.Array.Clone(), actionBuffersOut.ContinuousActions.Offset, actionBuffersOut.ContinuousActions.Length);
            var discreteSegment = new ActionSegment<int>((int[])actionBuffersOut.DiscreteActions.Array.Clone(), actionBuffersOut.DiscreteActions.Offset, actionBuffersOut.DiscreteActions.Length);
            actionMemory.PushBack(new ActionBuffers(continuousSegment, discreteSegment));
            var delayedAction = actionMemory[0];
            return delayedAction;
        }

        protected override void PostprocessActionBuffers(ActionBuffers actionBuffersOut)
        {
            
        }
    }
}
}
