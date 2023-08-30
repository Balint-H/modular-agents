using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

namespace Mujoco.Extensions
{
    public class GlobalImpactPerturbHandler : TrainingEventHandler
    {
        public override EventHandler Handler => (object sender, EventArgs e) => TogglePerturbation();

        [SerializeField]
        public MjBody rootBody;

        [SerializeField, Tooltip("If empty, the root's whole hierarchy will be candidates")]
        private List<MjBody> includeBodies;
        [SerializeField]
        private List<MjBody> excludeBodies;
        private List<MjBody> perturbedBodies;

        [SerializeField]
        PerturbType perturbationType;

        int curOrderedIndex;

        [SerializeField, Tooltip("Enforced by OnValidate to be positive")]
        float maxMagnitude;

        [SerializeField, Tooltip("Enforced by OnValidate to be positive and smaller than the max")]
        float minMagnitude;
            
            

        [SerializeField]
        bool scaleWithMass;

        [Serializable]
        private enum PerturbType
        {
            All,
            Ordered,
            Random
        }

        public bool IsPerturbationActive { get; private set; }

        private void Awake()
        {
            var candidateBodies = rootBody.GetComponentsInChildren<MjBody>();
            Func<MjBody, bool> inclusionCriteria = bd => (includeBodies == null || includeBodies.Count == 0) || includeBodies.Contains(bd);
            Func<MjBody, bool> exclusionCriteria = bd => excludeBodies.Contains(bd);

            perturbedBodies = candidateBodies.Where(bd => inclusionCriteria(bd) && !exclusionCriteria(bd)).ToList();
        }

        public void TogglePerturbation()
        {
            if(IsPerturbationActive)
            {
                ResetPerturbation();
            }
            else
            {
                Perturb();
            }
            IsPerturbationActive = !IsPerturbationActive;
        }

        public void Perturb()
        {
            switch(perturbationType)
            {
                case PerturbType.All:
                    PerturbAll();
                    return;

                case PerturbType.Ordered:
                    PerturbOrdered();
                    return;

                case PerturbType.Random:
                    PerturbRandom();
                    return;
            }
        }

        public void PerturbAll()
        {
            foreach (var bd in perturbedBodies)
            {
                PerturbBody(bd);
            }
        }
            
        public void PerturbRandom()
        {
            var randomIdx = UnityEngine.Random.Range(0, perturbedBodies.Count);
            PerturbBody(perturbedBodies[randomIdx]);
        }
            
        public void PerturbOrdered()
        {
            PerturbBody(perturbedBodies[curOrderedIndex]);
            curOrderedIndex = ++curOrderedIndex % perturbedBodies.Count;
        }

        public virtual void PerturbBody(MjBody body)
        {
            float perturbMagnitude = UnityEngine.Random.Range(minMagnitude, maxMagnitude);
            Vector3 randomForce = perturbMagnitude * UnityEngine.Random.onUnitSphere;

            if (scaleWithMass) MjState.ApplyScaledXForce(body, randomForce);
            else MjState.ApplyXForce(body, randomForce);
        }

        public virtual void ResetPerturbation()
        {
            foreach(var bd in perturbedBodies)
            {
                MjState.ApplyXForce(bd, Vector3.zero);
            }
        }

        private void OnValidate()
        {
            minMagnitude = Mathf.Max(minMagnitude, 0);
            maxMagnitude = Mathf.Max(maxMagnitude, 0);

            minMagnitude = Mathf.Min(minMagnitude, maxMagnitude);
        }

        private unsafe void OnDrawGizmosSelected()
        {
            if (!MjScene.InstanceExists) return;
            foreach(var bd in perturbedBodies)
            {
                var start = bd.transform.position;
                var frc = MjEngineTool.UnityVector3(MjScene.Instance.Data->xfrc_applied + 6 * bd.MujocoId);
                Gizmos.color = Color.red;
                if(frc.magnitude > 0) Gizmos.DrawRay(start, frc/100);
            }
        }
    }
    
}
