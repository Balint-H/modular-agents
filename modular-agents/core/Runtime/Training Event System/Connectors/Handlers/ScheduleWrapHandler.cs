using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScheduleWrapHandler : TrainingEventHandler
{
    [SerializeField]
    TrainingEvent conditionedEvent;

    [SerializeField]
    TrainingEventHandler wrappedEffect;

    private bool trigger;

    public override EventHandler Handler => SetTrigger;

    private void Awake()
    {
        conditionedEvent.SubscribeHandler(FireIfTriggered);
    }

    private void FireIfTriggered(object sender, EventArgs args)
    {
        if (!trigger) return;
        wrappedEffect.Handler.Invoke(sender, args);
        trigger = false;
    }

    public void SetTrigger(object sender, EventArgs args)
    {
        trigger = true;
    }

}
