using  ModularAgents.Kinematic;
using ModularAgents.Kinematic.Mujoco;

using Unity.MLAgents.Sensors;
using UnityEngine;

namespace ModularAgents.Throwing
{ 

public  class MjWalk2TargetObservationSource : ObservationSource
{

   
    [SerializeField]
    protected Transform simulationTransform;

  
    protected BodyChain simChain;

    [SerializeField]
    protected TargetHandler targetHandler;


    float maxScalingDistance = 10.0f;

    public override int Size => 4;

        public override void OnAgentStart()
        {
           
            simChain = new MjBodyChain(simulationTransform);

        }



        public override void FeedObservationsToSensor(VectorSensor sensor)
    {
        ReferenceFrame fKin = new ReferenceFrame(simChain.RootForward, simChain.CenterOfMass);

        Vector3 whereIsTarget = targetHandler.CurrentTargetPosition;

            //we consider that all the obects beyond 10 meters are at 10 meters
        if (   ( targetHandler.CurrentTargetPosition - simChain.Root.Position ).magnitude > maxScalingDistance )  
            whereIsTarget = simChain.Root.Position + ((targetHandler.CurrentTargetPosition - simChain.Root.Position).normalized * maxScalingDistance);

        sensor.AddObservation(fKin.WorldToCharacter(whereIsTarget));
        sensor.AddObservation(targetHandler.h);
    }


    private void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying) return;
        Gizmos.color = Color.white;

        ReferenceFrame fKin = new ReferenceFrame(simChain.RootForward, simChain.CenterOfMass);
        var localTarget = fKin.WorldToCharacter(targetHandler.CurrentTargetPosition);
        var globalStart = fKin.CharacterToWorld(Vector3.zero);
        Gizmos.DrawRay(globalStart, fKin.CharacterDirectionToWorld(localTarget));
    }


    }
}
