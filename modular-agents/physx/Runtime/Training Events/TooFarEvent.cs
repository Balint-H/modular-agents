using ManyWorlds;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Triggers an event when one of the objects are out of bounds for the training environment
/// </summary>
public class TooFarEvent : TrainingEvent
{
  

    [SerializeField]
    Transform referenceTransform;
    [SerializeField]
    Transform ragdollTransform;

    [SerializeField]
    float maxDistance;

    private bool AreTransformsWithinMaxDist { get => Vector3.Distance(referenceTransform.position,ragdollTransform.position) < maxDistance; }

    private void Update()
    {
        if (!AreTransformsWithinMaxDist) OnTrainingEvent(EventArgs.Empty);
    }

}
