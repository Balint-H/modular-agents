using ModularAgents.Kinematic;
using UnityEngine;

using ModularAgents.Kinematic.Mujoco;


namespace ModularAgents.Throwing
{ 
public class MjWalk2TargetRewardSource: RewardSource
{

    [SerializeField]
    Transform bodyRoot;//equivalent to projectile in ThrowRewardSource, but also used to find the direction forward

        [SerializeField]
        ModularAgent agent;

        IKinematic myBody;

    [SerializeField]
    TargetHandler target;

    [SerializeField]
    float rewardFallOff=-2.5f;

    [SerializeField]
     float targetSpeed = 1.2f;

   // [SerializeField]
   // float distanceOffset = 0f;

    public override float Reward => CalculateReward();

    public override void OnAgentStart()
    {
            myBody = bodyRoot.GetIKinematic();
    }

    float CalculateReward()
    {
            if (target.h)
            {
                if (agent.MaxStep > 0)
                    return (agent.MaxStep - agent.StepCount);
                else

                    return 10f;


            }
            else
            {

                /*
                //a naive attempt that makes the character turn too slowly, probably because distances play a role
                float d = (bodyRoot.position - target.CurrentTargetPosition).magnitude - distanceOffset;
                float dir = (Vector3.Dot(myBody.Forward.Horizontal3D(), (bodyRoot.position - bodyRoot.position).Horizontal3D()) + 1) / 2.0f;//value between 0 and 1 depending on the direction;


                if (d < 0) d = 0f;
                return Mathf.Exp(rewardFallOff * d * d) * (dir * 0.9f + 0.1f);
                */


                float tempValue = Vector3.Dot(myBody.Velocity, (target.CurrentTargetPosition - bodyRoot.position).normalized);

                //copying from Peng's DeepMimic:
                float d = targetSpeed - Vector3.Dot(myBody.Velocity, (target.CurrentTargetPosition - bodyRoot.position).normalized);
                if (d < 0) d = 0f;
                return Mathf.Exp(rewardFallOff * d * d);

            }
        }
}
}