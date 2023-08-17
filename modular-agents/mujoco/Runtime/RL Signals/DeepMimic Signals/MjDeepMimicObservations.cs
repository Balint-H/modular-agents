using Mujoco;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents.Sensors;
using UnityEngine;

using ModularAgents.Kinematic.Mujoco;


namespace ModularAgents.DeepMimic
{
    public class MjDeepMimicObservations : DeepMimicObservations
    {

        protected override IEnumerable<Transform> FilterTransforms(IEnumerable<Transform> transformCollection)
        {
            return transformCollection
                .Where(t => t.IsIKinematic())
                .Where(mjb => mjb.GetComponentInDirectChildren<MjBaseJoint>() && !mjb.GetComponentInDirectChildren<MjFreeJoint>())
                .Where(t => !t.name.Contains("toe", System.StringComparison.OrdinalIgnoreCase) && !t.name.Contains("neck", System.StringComparison.OrdinalIgnoreCase))
                .Select(mjb => mjb.transform);
        }

        public override void OnAgentStart()
        {
            root = rootTransform.GetIKinematic();
            observedKinematics = FilterTransforms(rootTransform.GetComponentsInChildren<Transform>()).Select(x => x.GetIKinematic() ).ToList();
        }
    }

}