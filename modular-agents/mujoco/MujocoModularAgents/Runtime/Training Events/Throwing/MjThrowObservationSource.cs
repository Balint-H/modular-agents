using ModularAgents.Kinematic.Mujoco;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace ModularAgents.Throwing
{ 
public  class MjThrowObservationSource : ThrowObservationSource
{

    //this needs to be defined for each case:
    public override void OnAgentStart()
    { 
        kinChain = new MjBodyChain(kinematicTransform);
        simChain = new MjBodyChain(simulationTransform);

    }
    


}
}
