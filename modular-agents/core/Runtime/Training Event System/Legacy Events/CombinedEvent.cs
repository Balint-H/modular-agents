using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

public class CombinedEvent : TrainingEvent
{
    [SerializeField]
    List<TrainingEvent> events;

    private void Awake()
    {
        foreach (var ev in events) ev.SubscribeHandler(WrapperEvent);
    }

    public void WrapperEvent(object sender, EventArgs e)
    {
        OnTrainingEvent(e);
    }
}