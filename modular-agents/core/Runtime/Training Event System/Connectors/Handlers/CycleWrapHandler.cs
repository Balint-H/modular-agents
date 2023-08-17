using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Each time this handler is called it will invoke the next handler in the list. For example, it is useful to set up successsive goals: walk to destination A, then 
/// TODO
/// </summary>
public class CycleWrapHandler : TrainingEventHandler
{
    int curHandlerIdx = 0;

    [SerializeField]
    List<TrainingEventHandler> handlers;

    public override EventHandler Handler => HandleAndCycle;

    public void HandleAndCycle(object sender, EventArgs e)
    {
        curHandlerIdx = (curHandlerIdx + 1) % handlers.Count;
        handlers[curHandlerIdx].Handler?.Invoke(sender, e);
    }
}
