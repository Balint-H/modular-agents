using  ModularAgents.Kinematic;
//using MjKinematic;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace ModularAgents.Throwing
{ 

public abstract class ThrowObservationSource : ObservationSource
{
    //[SerializeField]
    //DReConAgent agent;

    [SerializeField]
    protected Transform kinematicTransform;
    
    [SerializeField]
    protected Transform simulationTransform;

    protected BodyChain kinChain;
    protected BodyChain simChain;

    [SerializeField]
    protected TargetHandler target;

    public override int Size => 4;
   
    public override void FeedObservationsToSensor(VectorSensor sensor)
    {
        ReferenceFrame fKin = new ReferenceFrame(simChain.RootForward, simChain.CenterOfMass);

        sensor.AddObservation(fKin.WorldToCharacter(target.CurrentTargetPosition));
        sensor.AddObservation(target.h);
    }


        private void OnDrawGizmosSelected()
        {
            if (!Application.isPlaying) return;
            Gizmos.color = Color.white;

            ReferenceFrame fKin = new ReferenceFrame(simChain.RootForward, simChain.CenterOfMass);
            var localTarget = fKin.WorldToCharacter(target.CurrentTargetPosition);
            var globalStart = fKin.CharacterToWorld(Vector3.zero);
            Gizmos.DrawRay(globalStart, fKin.CharacterDirectionToWorld(localTarget));
        }


    }
}
