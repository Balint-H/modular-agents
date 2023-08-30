using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

using ModularAgents.Kinematic;


namespace ModularAgents.DeepMimic
{
    //does not implement the initialization because it has no access to the body chain
    public abstract class DeepMimicRewards : RewardSource
    {
        [SerializeField]
        protected Transform kinRoot;

        [SerializeField]
        protected Transform simRoot;

        protected BodyChain kinChain;
        protected BodyChain simChain;

        [SerializeField]
        protected List<Transform> kinEETransforms;
        protected IReadOnlyList<IKinematic> kinEEs;

        [SerializeField]
        protected List<Transform> simEETransforms;
        protected IReadOnlyList<IKinematic> simEEs;

        [SerializeField]
        bool useGlobalPositions = true;


        public override float Reward => CalculateRewards();

        public float CalculateRewards()
        {
            ReferenceFrame simFrame = new ReferenceFrame(simChain.RootForward, simChain.Root.Position); // Not CoM as then the CoM reward would have no meaning
            ReferenceFrame kinFrame = new ReferenceFrame(kinChain.RootForward, kinChain.Root.Position); // Not CoM as then the CoM reward would have no meaning


            return 0.65f * PoseReward(kinChain, simChain) +
                    0.1f * VelocityReward(kinChain, simChain) +
                    0.15f * EndEffectorReward(kinEEs, simEEs, kinFrame, simFrame, useGlobalPositions) +
                    0.1f * CetnerOfMassReward(kinChain, simChain, kinFrame, simFrame, useGlobalPositions);
        }

        private static float Sq(float a) => a * a;

        private static float PoseReward(BodyChain kinChain, BodyChain simChain)
        {
            float poseLoss = simChain.Zip(kinChain, (sim, kin) => Sq(Quaternion.Angle(sim.LocalRotation, kin.LocalRotation) * Mathf.Deg2Rad)).Sum();

            return Mathf.Exp(-2f * poseLoss / kinChain.Count);

        }

        private static float VelocityReward(BodyChain kinChain, BodyChain simChain)
        {
            float velocityLoss = simChain.Zip(kinChain, (sim, kin) => (sim.LocalAngularVelocity - kin.LocalAngularVelocity).sqrMagnitude).Sum();

            return Mathf.Exp(-0.1f * velocityLoss / kinChain.Count);
        }

        private static float EndEffectorReward(IReadOnlyList<IKinematic> kinEEs, IReadOnlyList<IKinematic> simEEs, ReferenceFrame kinFrame, ReferenceFrame simFrame, bool global)
        {
            float positionLoss = global ? simEEs.Zip(kinEEs, (sim, kin) => (sim.Position - kin.Position).sqrMagnitude).Sum() :
                                          simEEs.Zip(kinEEs, (sim, kin) => (simFrame.WorldToCharacter(sim.Position) - kinFrame.WorldToCharacter(kin.Position)).sqrMagnitude).Sum();

            return Mathf.Exp(-10 * positionLoss);
        }

        private static float CetnerOfMassReward(BodyChain kinChain, BodyChain simChain, ReferenceFrame kinFrame, ReferenceFrame simFrame, bool global)
        {
            float comLoss = global ? (simChain.CenterOfMass - kinChain.CenterOfMass).sqrMagnitude :
                                     (simFrame.WorldToCharacter(simChain.CenterOfMass) - kinFrame.WorldToCharacter(kinChain.CenterOfMass)).sqrMagnitude;

            return Mathf.Exp(-10f * comLoss);
        }


    /*    public override void OnAgentInitialize()
        {
            kinChain = kinRoot.GetBodyChain();
            simChain = simRoot.GetBodyChain();
            kinEEs = kinEETransforms.Select(x => x.GetKinematic()).ToList();
            simEEs = simEETransforms.Select(x => x.GetKinematic()).ToList();
        }*/

        /// <summary>
        /// Only intended for editor scripts.
        /// </summary>
        public void SetEndEffectors(IEnumerable<Transform> kinEEInput, IEnumerable<Transform> simEEInput)
        {
            kinEETransforms = kinEEInput.ToList();
            simEETransforms = simEEInput.ToList();
        }
        public Transform KinRoot => kinRoot;
        public Transform SimRoot => simRoot;
    }

}