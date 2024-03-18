using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ModularAgents.Kinematic.PhysX;
using ModularAgents.Kinematic;
namespace ModularAgents.Throwing
{ 
public class PhysXThrowObservationSource : ThrowObservationSource
{

    public override void OnAgentStart()
    {
        kinChain = new PhysXBodyChain(kinematicTransform);
        simChain = new PhysXBodyChain(simulationTransform);
    }

    private void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying) return;
        Gizmos.color = Color.white;
        ReferenceFrame fKin = new ReferenceFrame(kinChain.RootForward, simChain.CenterOfMass);
        var localTarget = fKin.WorldToCharacter(target.transform.position);
        var globalStart = fKin.CharacterToWorld(Vector3.zero);
        Gizmos.DrawRay(globalStart, fKin.CharacterDirectionToWorld(localTarget));
    }

}
}
