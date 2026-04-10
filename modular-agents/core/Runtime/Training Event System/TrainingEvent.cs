using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;

/// <summary>
/// A flexible base class that can be inherited for interfacing with different handlers
/// </summary>
public abstract class TrainingEvent : MonoBehaviour
{

    protected event EventHandler eventHandler;
    public virtual void SubscribeHandler(EventHandler subscriber)
    {
        eventHandler += subscriber;
    }

    public virtual void UnsubscribeHandler(EventHandler subscribed)
    {
        eventHandler -= subscribed;
    }

    protected virtual void OnTrainingEvent(EventArgs e)
    {
        eventHandler?.Invoke(this, e);
    }


    public void ManuallyTrigger(EventArgs e)
    {

        if (!Application.isPlaying)
        {
            foreach(var eventHandler in triggeredHandlers)
            {
                eventHandler.Handler?.Invoke(this, e);
            }
        }
        else
        {
            OnTrainingEvent(e);
        }
    }

    [SerializeField, Tooltip("All handlers in this list will subscribe to this event. Can be empty if you subscribe to this event elsewhere (e.g. an EventBuilder).")]
    [FormerlySerializedAs("defaultHandlers")]
    public List<TrainingEventHandler> triggeredHandlers;

    [SerializeField, HideInInspector]
    private List<TrainingEventHandler> _triggeredHandlers;  // We'll use this buffer to check which elements changed in OnValidate

    // Note that if your child class training event overrides OnEnable, it should call SubscribeSerializedHandlers itself. Alternatively you can override InitializeEvent().
    protected virtual void InitializeEvent()
    {

    }

    /// <summary>
    /// If you override this, make sure to call SubscribeSerializeHandlers().
    /// </summary>
    private void OnEnable ()
    {
        InitializeEvent();
        SubscribeSerializedHandlers();
    }

    protected void SubscribeSerializedHandlers()
    {
        if (triggeredHandlers == null) return;
        foreach (var handler in triggeredHandlers)
        {
            SubscribeHandler(handler.Handler);
        }
    }

    private void OnDisable()
    {
        eventHandler = null;
    }

    private void OnValidate()
    {
        if(triggeredHandlers == null) return;
        for (int i = 0; i < triggeredHandlers.Count; i++)
        {
            var handlerToCheckDuplicatesFor = triggeredHandlers[i];
            for (int j = i + 1; j < triggeredHandlers.Count; j++)
            {
                if (triggeredHandlers[j] == handlerToCheckDuplicatesFor) triggeredHandlers[j] = null;
            }
        }


        foreach (var trainingEventHandler in triggeredHandlers.Where(h => h !=null && !_triggeredHandlers.Contains(h)))
        {
            if (trainingEventHandler.triggeringEvents == null) trainingEventHandler.triggeringEvents = new List<TrainingEvent>();
            if (!trainingEventHandler.triggeringEvents.Contains(this)) trainingEventHandler.triggeringEvents.Add(this);
        }
        foreach (var trainingEvent in _triggeredHandlers.Where(h => h != null && !triggeredHandlers.Contains(h)))
        {
            if (trainingEvent.triggeringEvents.Contains(this)) trainingEvent.triggeringEvents.Remove(this);
        }  

        _triggeredHandlers.Clear();
        for(int i=0; i<triggeredHandlers.Count; i++) 
        {
            _triggeredHandlers.Add(triggeredHandlers[i]);
        }
    }

}
