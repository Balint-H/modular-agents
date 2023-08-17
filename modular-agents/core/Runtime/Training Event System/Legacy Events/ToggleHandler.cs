using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ToggleHandler : TrainingEventHandler
{
    private int nextEvent;

    [SerializeField]
    List<TrainingEventHandler> handlers;

    public override EventHandler Handler => (object sender, EventArgs e) => InvokeAndIncrement(sender, e);

    public void InvokeAndIncrement(object sender, EventArgs e)
    {
        if (handlers.Count == 0) return;
        handlers[nextEvent].Handler?.Invoke(sender, e);
        nextEvent = ++nextEvent % handlers.Count;
    }

    public void InvokeAndIncrement()
    {
        if (handlers.Count == 0) return;
        handlers[nextEvent].Handler?.Invoke(this, EventArgs.Empty);
        nextEvent = ++nextEvent % handlers.Count;
    }
}
