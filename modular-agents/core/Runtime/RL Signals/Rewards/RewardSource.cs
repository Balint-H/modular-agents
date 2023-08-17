using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;


public abstract class RewardSource: MonoBehaviour
{
    public abstract float Reward { get;}

    public abstract void OnAgentStart();
}

[Serializable]
public class WeightedRewardSource
{
    [SerializeField]
    private RewardSource source;
    [SerializeField]
    private float weight;

    public float Weight { get => weight; }
    public float Reward { get => source.Reward; }

    public string SourceName { get => source.name; }


    public void OnAgentStart()
    {
        source.OnAgentStart();
    }


    public bool IsEmpty()
    {
        return source == null;
    }
}




