using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;

/// <summary>
/// A flexible base class that can supply a handling function when paired up with a TrainingEvent
/// </summary>
public abstract class TrainingEventHandler : MonoBehaviour
{
    public abstract EventHandler Handler { get; }

    [SerializeField, Tooltip("This handler will subscribe to all events in this list. Can be empty if you subscribe this handler elsewhere (e.g. an EventBuilder).")]
    [FormerlySerializedAs("defaultEvents")]
    public List<TrainingEvent> triggeringEvents;

    [SerializeField, HideInInspector]
    private List<TrainingEvent> _triggeringEvents;  // We'll use this buffer to check which elements changed in OnValidate


    private void OnValidate()
    {
        if (triggeringEvents == null) return;
        for (int i = 0; i < triggeringEvents.Count; i++)
        {
            var eventToCheckDuplicatesFor = triggeringEvents[i];
            for (int j = i + 1; j < triggeringEvents.Count; j++)
            {
                if (triggeringEvents[j] == eventToCheckDuplicatesFor) triggeringEvents[j] = null;
            }
        }

        foreach (var trainingEvent in triggeringEvents.Where(e => e != null && !_triggeringEvents.Contains(e)))
        {
            if (trainingEvent.triggeredHandlers == null) trainingEvent.triggeredHandlers = new List<TrainingEventHandler> ();
            if (!trainingEvent.triggeredHandlers.Contains(this)) trainingEvent.triggeredHandlers.Add(this);
        }
        foreach (var trainingEvent in _triggeringEvents.Where(e => e != null && !triggeringEvents.Contains(e)))
        {
            if (trainingEvent.triggeredHandlers.Contains(this)) trainingEvent.triggeredHandlers.Remove(this);
        }
        

        _triggeringEvents.Clear();
        for (int i = 0; i < triggeringEvents.Count; i++)
        {
            _triggeringEvents.Add(triggeringEvents[i]);
        }
    }
}
