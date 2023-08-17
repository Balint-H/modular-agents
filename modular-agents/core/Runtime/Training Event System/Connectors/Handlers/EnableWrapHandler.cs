using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EnableWrapHandler
    : TrainingEventHandler
{
    [SerializeField]
    TrainingEvent conditionedEvent;

    [SerializeField]
    TrainingEventHandler wrappedEffect;

    private bool isEnabled;

    public override EventHandler Handler => EnableConditionedEvent;

    private void Awake()
    {
        conditionedEvent.SubscribeHandler(FireIfTriggered);
    }

    private void FireIfTriggered(object sender, EventArgs args)
    {
        if (!isEnabled) return;
        wrappedEffect.Handler.Invoke(sender, args);
        isEnabled = false;
    }

    public void EnableConditionedEvent(object sender, EventArgs args)
    {
        isEnabled = true;
    }

}
