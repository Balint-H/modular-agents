using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System;
using ModularAgents.Kinematic;

namespace ModularAgents.DReCon
{ 
    /// <summary>
    /// Following Bergamin et al. 2019. Using square differences seems to lead to higher fidelity motions, toggleable through the useSquareLoss flag.
    /// This class is abstract to allow the two different Physics engine specific implementations (PhysX and MuJoCo) use shared logic.
    /// </summary>
    public abstract class DReConRewardSource : RewardSource
    {
        [SerializeField]
        protected Transform kinematicTransform;

        [SerializeField]
        protected Transform simulationTransform;

        [SerializeField]
        protected GameObject kinematicHead;

        [SerializeField]
        protected GameObject simulationHead;

        [SerializeField]
        private bool useHeightOnly;

        [SerializeField]
        bool useSquareLoss;

        protected IKinematic kinHead;
        protected IKinematic simHead;

        protected BoundingBoxChain kinChain;
        protected BoundingBoxChain simChain;

        protected int nBodies;

        public override float Reward { get => CalculateReward(); }

   
        private float CalculateReward()
        {
           // Debug.Log(kinChain.Root.Name);
            ReferenceFrame fKin = new ReferenceFrame(kinChain.RootForward, kinChain.CenterOfMass);
            ReferenceFrame fSim = new ReferenceFrame(kinChain.RootForward, simChain.CenterOfMass); // Same orientation, different origin

            float eFall = useHeightOnly ? Mathf.Clamp01(1.3f - 1.4f * Mathf.Abs(simHead.CenterOfMass.y - kinHead.CenterOfMass.y)) : Mathf.Clamp01(1.3f - 1.4f * (fSim.WorldToCharacter(simHead.CenterOfMass) - fKin.WorldToCharacter(kinHead.CenterOfMass)).magnitude);

            float positionDiff, velocityDiff, localPoseDiff, comVDiff;
            float reward;
            if (!useSquareLoss)
            {
                (positionDiff, velocityDiff, localPoseDiff) = BoundingBoxChain.CalulateDifferences(kinChain, fKin, simChain, fSim);
                comVDiff = (fKin.WorldDirectionToCharacter(kinChain.CenterOfMassVelocity) - fSim.WorldDirectionToCharacter(simChain.CenterOfMassVelocity)).magnitude;

                reward = eFall * (Mathf.Exp(-10f / nBodies * positionDiff)      //-7.37 in old implementation
                                + Mathf.Exp(-0.1f / nBodies * velocityDiff)       //-1 in old implementation
                                + Mathf.Exp(-10f / nBodies * localPoseDiff)    //-6.5 in old implementation
                                + Mathf.Exp(-comVDiff));

            }
            else
            {
                (positionDiff, velocityDiff, localPoseDiff) = BoundingBoxChain.CalulateSquareDifferences(kinChain, fKin, simChain, fSim);
                comVDiff = (fKin.WorldDirectionToCharacter(kinChain.CenterOfMassVelocity) - fSim.WorldDirectionToCharacter(simChain.CenterOfMassVelocity)).magnitude;
                comVDiff *= comVDiff; //in old implementation

                reward = eFall * (Mathf.Exp(-7.37f / nBodies * positionDiff)      //-7.37 in old implementation
                                + Mathf.Exp(-1f / nBodies * velocityDiff)       //-1 in old implementation
                                + Mathf.Exp(-6.5f / nBodies * localPoseDiff)    //-6.5 in old implementation
                                + Mathf.Exp(-comVDiff));
            }
            return reward;
        }

        private void OnDrawGizmosSelected()
        {
            if (!Application.isPlaying) return;
            ReferenceFrame fKin = new ReferenceFrame(kinChain.RootForward, kinChain.CenterOfMass);
            ReferenceFrame fSim = new ReferenceFrame(kinChain.RootForward, simChain.CenterOfMass);

            fKin.Draw();
            fSim.Draw();

            kinChain.Draw();
            simChain.Draw();

        

            foreach (int idx in Enumerable.Range(0, kinChain.ColliderCount).Where((x, i) => i % 1 == 0))
            {
                Gizmos.color = Color.cyan;

                kinChain.DrawVelocities(fKin, idx);

                Gizmos.color = Color.magenta;

                simChain.DrawVelocities(fSim, idx);

                Gizmos.color = Color.clear;

                //BoundingBoxChain.DrawPositionalDifferences(kinChain, fKin, simChain, fSim, idx);
                BoundingBoxChain.DrawVelocityDifferences(simChain, fSim, kinChain, fKin, idx);
            }

        
        }

        public GameObject KinematicHead { set => kinematicHead = value; }
        public GameObject SimulationHead { set => simulationHead=value; }

        public Transform KinematicTransform { get => kinematicTransform; }
        public Transform SimulationTransform { get => simulationTransform; }

    }
}
