using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Triggered when the two transforms referenced are farther apart than maxDistance in metres.
/// </summary>

public class PositionsDivergedEvent : TrainingEvent
{
    [SerializeField]
    private Transform positionA;

    [SerializeField]
    private Transform positionB;

    [SerializeField]
    private float maxDistance;

    private void Update()
    {
        if ((positionA.position - positionB.position).magnitude > maxDistance) OnTrainingEvent(EventArgs.Empty);
    }
}
