using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Mujoco;

namespace Examples.Cartpole
{
    public class CartpoleActuator : ActuatorComponent
    {
        // Reference value for the overall ActionSpec of the combined IActuators associated with the component
        public override ActionSpec ActionSpec => new ActionSpec(numContinuousActions:1);

        // We need to add the actuator from the Editor
        [SerializeField]
        private MjActuator actuator;

        
        private ControlledMjActuator controlledMjActuator; // Wrapper object that applies actions from the Agent to the MjActuator.

        public override IActuator[] CreateActuators()
        {
            controlledMjActuator ??= new ControlledMjActuator(actuator);
            
            return new[] { controlledMjActuator}; // Could create and return multiple IActuators if needed
        }

        private void Start()
        {
            controlledMjActuator ??= new ControlledMjActuator(actuator);
            MjScene.Instance.ctrlCallback += controlledMjActuator.ApplyControl;
        }

        private class ControlledMjActuator : IActuator
        {
            private ActionSpec actionSpec;
            public ActionSpec ActionSpec { get => actionSpec; }

            private MjActuator wrappedActuator;

            private double nextCtrl;

            public string Name => wrappedActuator.name;


            // Used when no model is connected to the BrainParameters component.
            public void Heuristic(in ActionBuffers actionBuffersOut) { } 


            // The agent distributes its actions among all of its IActuators, each receiving a segment based on their ActionSpecs.
            public void OnActionReceived(ActionBuffers actionBuffers)
            {
                nextCtrl = actionBuffers.ContinuousActions[0];
            }

            public unsafe void ApplyControl(object sender, MjStepArgs e)
            {
                e.data->ctrl[wrappedActuator.MujocoId] = nextCtrl;
                wrappedActuator.Control = (float) nextCtrl;
            }


            // Called at the end of an episode.
            public void ResetData() { } 


            // Limit the number of possible actions if taking discrete actions (e.g. prevent gridworld agent from walking into a wall), not applicable
            public void WriteDiscreteActionMask(IDiscreteActionMask actionMask){ } 


            public ControlledMjActuator(MjActuator actuator)
            {
                actionSpec = new ActionSpec(numContinuousActions: 1);
                wrappedActuator = actuator;
            }
    
        }

    }
}
