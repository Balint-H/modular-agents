using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Unity.MLAgents.Actuators;

namespace ModularAgents.MotorControl
{ 
public abstract class ActuatorComponentWrapper: ActuatorComponent
{
    [SerializeField]
    protected ActuatorComponent wrappedActuatorComponent;


    IActuator[] actuators;

    public abstract IActuator WrapActuator(IActuator actuator);

    public override IActuator[] CreateActuators()
    {
        if (actuators == null)
        {
            actuators = wrappedActuatorComponent.CreateActuators().Select(a => WrapActuator(a)).ToArray();
        }
        return actuators;
    }

   

}

public abstract class ActuatorWrapper : IActuator
{
    protected IActuator wrappedActuator;


    public virtual ActionSpec ActionSpec => wrappedActuator.ActionSpec;

    protected abstract string WrappingName { get; }

    public string Name => WrappingName+"_"+wrappedActuator.Name;

    protected abstract ActionBuffers PreprocessActionBuffers(ActionBuffers actionBuffersOut);
    protected abstract void PostprocessActionBuffers(ActionBuffers actionBuffersOut);

    public virtual void Heuristic(in ActionBuffers actionBuffersOut)
    {
        wrappedActuator.Heuristic(actionBuffersOut);
    }

    public void OnActionReceived(ActionBuffers actionBuffers)
    {
        actionBuffers = PreprocessActionBuffers(actionBuffers);
        wrappedActuator.OnActionReceived(actionBuffers);
        PostprocessActionBuffers(actionBuffers);
    }

    protected abstract void ResetWrapperData();

    public void ResetData()
    {
        ResetWrapperData();
        wrappedActuator.ResetData();
    }

    public void WriteDiscreteActionMask(IDiscreteActionMask actionMask)
    {
        wrappedActuator.WriteDiscreteActionMask(actionMask);
    }
}
}