using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Linq;
using ModularAgents.Kinematic.PhysX;



namespace ModularAgents.DeepMimic
{ 
public class PhysXDeepMimicObservations : DeepMimicObservations
{

        [SerializeField]
        protected Animator animator;

        protected override IEnumerable<Transform> FilterTransforms(IEnumerable<Transform> transformCollection)
    {
        return transformCollection
            .Where(t => t.IsIKinematic())
            .Where(mjb => mjb.GetComponentInDirectChildren<ArticulationBody>())
            .Where(t => !t.name.Contains("toe", System.StringComparison.OrdinalIgnoreCase) && !t.name.Contains("neck", System.StringComparison.OrdinalIgnoreCase))
            .Select(mjb => mjb.transform);
            //TODO: shoould we remove the root?


    }

    public override void OnAgentStart()
    {
        root = rootTransform.GetIKinematic();
        observedKinematics = FilterTransforms(rootTransform.GetComponentsInChildren<Transform>()).Select(x => x.GetIKinematic()).ToList();
         
    }


    protected override float GetPhase()
    {
        return animator.GetCurrentAnimatorStateInfo(0).normalizedTime;
    }

}
}


