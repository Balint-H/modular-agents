using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class EventBuilder : MonoBehaviour
{
    [SerializeField]
    List<HandledEvent> handledEvents;

    bool isActive;

    private void ActivateEvents()
    {
        if (isActive) return;
        foreach (HandledEvent handledEvent in handledEvents)
        {
            handledEvent.Activate();
        }
        isActive = true;
    }
    private void DeactivateEvents()
    {
        if (!isActive) return;
        foreach (HandledEvent handledEvent in handledEvents)
        {
            handledEvent.Deactivate();
        }
        isActive = false;
    }

    private void Awake()
    {
        if (!isActiveAndEnabled) return;
        ActivateEvents();
    }

    private void OnEnable()
    {
        ActivateEvents();
    }

    private void OnDisable()
    {
        DeactivateEvents();
    }

}

[Serializable]
public class HandledEvent
{
    [SerializeField]
    TrainingEvent trainingEvent;

    [SerializeField]
    TrainingEventHandler effect;

    public void Activate()
    {
        trainingEvent.SubscribeHandler(effect.Handler);
    }

    public void Deactivate()
    {
        trainingEvent.UnsubscribeHandler(effect.Handler);
    }


}
