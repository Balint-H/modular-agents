using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConstantRewardSource : RewardSource
{
    [SerializeField]
    float reward;

    public override float Reward => reward;

    public override void OnAgentStart()
    {

    }
}
