using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ModularAgents.Throwing
{ 
public class ThrowRewardSource : RewardSource
{

    [SerializeField]
    Transform projectile;

    [SerializeField]
    TargetHandler target;

    [SerializeField]
    float rewardFallOff=-4f;

    [SerializeField]
    float distanceOffset = 0f;

    public override float Reward => CalculateReward();

    public override void OnAgentStart()
    {

    }

    // Start is called before the first frame update
    float CalculateReward()
    {
        if (target.h) return 1f;
        else 
        {
            var d = (projectile.position - target.CurrentTargetPosition).magnitude - distanceOffset;
            if (d < 0) d = 0f;
            return Mathf.Exp(rewardFallOff * d * d);
        }
    }
}
}