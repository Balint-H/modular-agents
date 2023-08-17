using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

using System;

namespace ModularAgents.DReCon
{ 
public abstract class DReConActuator: MonoBehaviour
{
    public abstract int ActionSpaceSize { get; }
    public abstract void ApplyActions(float[] actions, float actionTimeDelta);

    public abstract float[] GetActionsFromState();

    public virtual void OnAgentInitialize(DReConAgent agent = null) { }
}
}