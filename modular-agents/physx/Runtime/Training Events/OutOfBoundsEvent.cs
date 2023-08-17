using ManyWorlds;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Triggers an event when one of the objects are out of bounds for the training environment
/// </summary>
public class OutOfBoundsEvent : TrainingEvent
{
    [SerializeField]
    SpawnableEnv spawnableEnv;

    [SerializeField]
    List<Transform> trackedTransforms;

    private bool AreAllTransformsInBounds { get => trackedTransforms.TrueForAll(t => spawnableEnv.IsPointWithinBoundsInWorldSpace(t.position + new Vector3(0f, .1f, 0f))); }

    private void Update()
    {
        if (!AreAllTransformsInBounds) OnTrainingEvent(EventArgs.Empty);
    }

}
