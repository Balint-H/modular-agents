using Mujoco;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class BodyNameMatcher : MonoBehaviour
{
    [SerializeField]
    Transform renamedRoot;

    [SerializeField]
    Transform sourceRoot;

    [SerializeField]
    bool mjBodyOnly;

    Dictionary<Transform, string> originalNames;

    public void RenameBodiesToMatchClosest()
    {
        var renamedTransforms = renamedRoot.GetComponentsInChildren<Transform>();
        originalNames = new Dictionary<Transform, string>(renamedTransforms.Select(t => new KeyValuePair<Transform, string>(t, t.name)));
        Debug.Log($"Renaming : {string.Join(", ", renamedTransforms.Select(t => t.name))}");
        var source = mjBodyOnly ? sourceRoot.GetComponentsInChildren<MjBody>().Select(b => b.transform) : sourceRoot.GetComponentsInChildren<Transform>();
        foreach (Transform childTransform in source)
        {
            Func<Transform, float> Dist = (Transform t) => Vector3.Distance(t.position, childTransform.position);
            var closest = renamedTransforms.Aggregate((curMin, x) => Dist(x) < Dist(curMin) ? x : curMin);
            Debug.Log($"Closest to {childTransform.name} is {closest.name}");
            closest.name = childTransform.name;
            
        }
    }

    public void RestoreNames()
    {
        foreach (KeyValuePair<Transform, string> originalName in originalNames)
        {
            originalName.Key.name = originalName.Value;
        }
    }
}
