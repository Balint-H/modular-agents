using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using Unity.MLAgents.Actuators;
using System;
using ModularAgents.MotorControl;
using System.Linq;

namespace ModularAgents.MotorControl.Mujoco
{
    public class SPDGainActions : ActuatorComponent
    {

        [SerializeField]
        SPDActuatorComponent manipulatedActuator;

        /// <summary>
        /// If no joints added to subset, all joints from actuator will be considered
        /// </summary>
        [SerializeField]
        List<ModulationSubset> modulatedJointSubsets;

        [SerializeField]
        GainModulationMode gainModulationMode;  

        public override ActionSpec ActionSpec => ActionSpec.MakeContinuous(modulatedJointSubsets.Count * (gainModulationMode == GainModulationMode.Both? 2 : 1));

        public IEnumerable<IActuator> CriteriaToActuators()
        {
            foreach(var modulatedJointSubset in modulatedJointSubsets)
            {
                Func<MjBaseJoint, bool> InclusionCriteria = (MjBaseJoint j) => modulatedJointSubset.jointSubset.Count == 0 || modulatedJointSubset.jointSubset.Contains(j);
                Func<MjBaseJoint, bool> ExclusionCriteria = (MjBaseJoint j) => modulatedJointSubset.excludeSubset.Contains(j);

                var validJointsInActuator = manipulatedActuator.ActuatedJoints.Where(j => !ExclusionCriteria(j)).Where(j => InclusionCriteria(j));

                yield return new GainModulator(manipulatedActuator, validJointsInActuator.ToList(), gainModulationMode, modulatedJointSubset.stiffnessScale, modulatedJointSubset.dampingScale);
            }
        }

        public override IActuator[] CreateActuators()
        {
            return CriteriaToActuators().ToArray();
        }

        [Serializable]
        private class ModulationSubset
        {
            [SerializeField]
            public double stiffnessScale;

            [SerializeField]
            public double dampingScale;

            [SerializeField]
            public List<MjBaseJoint> jointSubset;

            [SerializeField]
            public List<MjBaseJoint> excludeSubset;
        }


        [Serializable]
        private enum GainModulationMode
        {
            Stiffness,
            Damping,
            Both
        }
        private class GainModulator : IActuator
        {
            public ActionSpec ActionSpec => ActionSpec.MakeContinuous( modulationMode == GainModulationMode.Both? 2 : 1);

            SPDActuatorComponent actuatorToModulate;
            int[] indices;
            List<MjBaseJoint> joints;
            bool mjInstanceHasStarted;
            GainModulationMode modulationMode;

            double stiffnessScale;
            double dampingScale;

            public GainModulator(SPDActuatorComponent actuatorToModulate, List<MjBaseJoint> joints, GainModulationMode modulationMode, double stiffnessScale=1, double dampingScale=1)
            {
                this.actuatorToModulate = actuatorToModulate;
                this.joints = joints;

                this.modulationMode = modulationMode;

                this.stiffnessScale = stiffnessScale;
                this.dampingScale = dampingScale;

                
            }

            public string Name => $"GainModulator_{joints[0].name}";

            public void Heuristic(in ActionBuffers actionBuffersOut)
            {
                
            }

            public void OnActionReceived(ActionBuffers actionBuffers)
            {
                if (!MjScene.InstanceExists) return;
                if (!mjInstanceHasStarted)
                {
                    var dofAdr = IMjJointState.GetDofAddresses(actuatorToModulate.ActuatedJoints).ToList();

                    indices = IMjJointState.GetDofAddresses(joints).Select(addr => dofAdr.IndexOf(addr)).ToArray();
                    UnityEngine.Assertions.Assert.IsTrue(indices.Length == indices.Distinct().Count());
                    mjInstanceHasStarted = true;
                }

                var actions = actionBuffers.ContinuousActions.Array[actionBuffers.ContinuousActions.Offset..(actionBuffers.ContinuousActions.Offset + actionBuffers.ContinuousActions.Length)];
                double stiffness, damping;
                double stiffnessBias = actuatorToModulate.PosGain;
                double dampingBias = actuatorToModulate.VelGain;
                
                switch(modulationMode)
                {
                    case GainModulationMode.Stiffness:
                        stiffness = stiffnessBias + stiffnessScale * actions[0];
                        SetStiffness(stiffness);
                        break;

                    case GainModulationMode.Damping:
                        damping = dampingBias + dampingScale * actions[0];
                        SetDamping(damping);
                        break;

                    case GainModulationMode.Both:
                        stiffness = stiffnessBias + stiffnessScale * actions[0];
                        damping = dampingBias + dampingScale * actions[1];
                        SetStiffness(stiffness);
                        SetDamping(damping);
                        break;
                } 
            }

            private void SetStiffness(double stiffness)
            {
                foreach (int idx in indices)
                {
                    actuatorToModulate.PosGainMatrix[idx, idx] = stiffness;
                }
            }

            private void SetDamping(double damping)
            {
                foreach (int idx in indices)
                {
                    actuatorToModulate.VelGainMatrix[idx, idx] = damping;
                }
            }

            public void ResetData()
            {
                
            }

            public void WriteDiscreteActionMask(IDiscreteActionMask actionMask)
            {
                
            }
        }
    }
}