using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


public class AgentEventArgs : EventArgs
{
    public float[] actions;
    public float reward;

    public AgentEventArgs(float[] actions, float reward)
    {
        this.actions = actions;
        this.reward = reward;
    }

    new public static AgentEventArgs Empty => new AgentEventArgs(new float[0], 0f);

}

/// <summary>
/// Deprecated interface, use ModularAgent instead. Let us know if you think this makes sense to keep.
/// </summary>
public interface IEventsAgent
{
    public event EventHandler<AgentEventArgs> OnPostAction;
    public event EventHandler<AgentEventArgs> OnBegin;
    public event EventHandler<AgentEventArgs> OnStart;
}