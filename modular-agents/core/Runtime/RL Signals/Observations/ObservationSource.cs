using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Sensors;
using UnityEngine;

public abstract class ObservationSource: MonoBehaviour
{
    public abstract void FeedObservationsToSensor(VectorSensor sensor);
    public abstract void OnAgentStart();
    public virtual int Size { get; }
}
