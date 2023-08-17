using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AgentActionPrinter : TrainingEventHandler
{
    public override EventHandler Handler => PrintActions;

    private void PrintActions(object sender, EventArgs args)
    {
        var agentArgs = args as AgentEventArgs;
        Debug.Log(String.Join(", ", agentArgs.actions));
    }
}
