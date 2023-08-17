using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Actuators;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;


namespace ModularAgents.MotorControl
{ 
    public class SmoothedActuatorComponent : ActuatorComponentWrapper, IRememberPreviousActions
    {
        [SerializeField]
        private float smoothingFactor;

        public override ActionSpec ActionSpec => wrappedActuatorComponent.ActionSpec;

        public float[] PreviousActions => smoothedActuator.PreviousActions;

        public int RememberedActionSize => ActionSpec.NumContinuousActions;

        SmoothedActuator smoothedActuator;

        public void SetPreviousActions(float[] actions)
        {
            smoothedActuator.SetPreviousActions(actions);
        }

        public override IActuator WrapActuator(IActuator actuator)
        {
            smoothedActuator = new SmoothedActuator(actuator, smoothingFactor, this);
            return smoothedActuator;
        }

        private class SmoothedActuator : ActuatorWrapper
        {
            protected override string WrappingName => "Smoothed";

            private SmoothedActuatorComponent component;

            Vector<float> smoothedActions;
            float smoothingFactor;

            public SmoothedActuator(IActuator actuator, float smoothingFactor)
            {
                wrappedActuator = actuator;
                this.smoothingFactor = smoothingFactor;
            
            }
            public SmoothedActuator(IActuator actuator, float smoothingFactor, SmoothedActuatorComponent component)
            {
                wrappedActuator = actuator;
                this.smoothingFactor = smoothingFactor;
                this.component = component;

            }


            public float[] PreviousActions => smoothedActions.ToArray();

            protected override void PostprocessActionBuffers(ActionBuffers actionBuffersOut)
            {

            }

            Vector<float> GetActuatorSpecificActionSegment(ActionBuffers actionBuffers)
            {
                var specificActionSegmentRange = actionBuffers.ContinuousActions.Offset..(actionBuffers.ContinuousActions.Offset + actionBuffers.ContinuousActions.Length);
                return Vector<float>.Build.Dense(actionBuffers.ContinuousActions.Array[specificActionSegmentRange]);
            }

            protected override ActionBuffers PreprocessActionBuffers(ActionBuffers actionBuffersOut)
            {
                CheckFactor();

                var incomingActions = GetActuatorSpecificActionSegment(actionBuffersOut);
                if (smoothedActions == null)
                {
                    smoothedActions = incomingActions;
                }
                else
                {
                    smoothedActions = smoothingFactor * smoothedActions + (1 - smoothingFactor) * incomingActions;
                }
                for(int i=0; i<smoothedActions.Count; i++)
                {
                    actionBuffersOut.ContinuousActions.Array[i + actionBuffersOut.ContinuousActions.Offset] = smoothedActions[i];
                }
                return actionBuffersOut;
            }

            private void CheckFactor()
            {
                if (!component) return;
                smoothingFactor = component.smoothingFactor;
            }

            protected override void ResetWrapperData()
            {
                smoothedActions = null;
            }

            public void SetPreviousActions(float[] actions)
            {
                smoothedActions = Vector<float>.Build.Dense(actions); 
            }
        }
    }


    public interface IRememberPreviousActions
    {
        public float[] PreviousActions { get; }
        public int RememberedActionSize { get; }
        public void SetPreviousActions(float[] actions);
    }
}