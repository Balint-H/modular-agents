using System.Collections;
using System.Collections.Generic;
using ModularAgents.Kinematic.PhysX;
using UnityEngine;

namespace ModularAgents.DReCon
{ 
public class PhysXDReConRewardSource : DReConRewardSource
{
    public override void OnAgentStart()
    {
        //Debug.Log("Base method called!");



        kinChain = new BoundingBoxChain(new PhysXBodyChain(kinematicTransform));
        simChain = new BoundingBoxChain(new PhysXBodyChain(simulationTransform));

        kinHead = kinematicHead.transform.GetIKinematic();

        simHead = simulationHead.transform.GetIKinematic();

        nBodies = kinChain.ColliderCount;
    }

}
}
