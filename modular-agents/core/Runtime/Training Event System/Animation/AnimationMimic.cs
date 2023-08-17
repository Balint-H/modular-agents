using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

namespace ModularAgents.TrainingEvents
{ 
/// <summary>
/// Directly maps motion from one kinematic chain to another (e.g. corresponding to a skinned mesh). Works only if no retargeting is needed i.e. the two ragdolls match in layout. 
/// A TrainingEvent is invoked after the kinematic chain is updated.
/// </summary>
public class AnimationMimic : TrainingEvent
{
    [SerializeField]
    Transform animatedHierarchyRoot;

    [SerializeField]
    Transform sourceHierarchyRoot;

    [SerializeField]
    string prefix;

    IReadOnlyList<Tuple<Transform, Transform>> pairedTransforms; // Source, Animated


    void Awake()
    {
        var candidateSources = sourceHierarchyRoot.GetComponentsInChildren<Transform>().ToList();
        pairedTransforms = animatedHierarchyRoot.GetComponentsInChildren<Transform>().Select(at => Tuple.Create(candidateSources.FirstOrDefault(ct => Utils.SegmentName(ct.name) == prefix+Utils.SegmentName(at.name)), at)).Where(tup => tup.Item1 != null).ToList();
    }

    void LateUpdate()
    {
        pairedTransforms[0].Item2.position = pairedTransforms[0].Item1.position;

        foreach((var source, var target) in pairedTransforms)
        {
            target.rotation = source.rotation;
        }

        OnTrainingEvent(EventArgs.Empty);
    }
}
}