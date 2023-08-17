using ModularAgents;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Mujoco;

public class ShowRewards : RewardSignal
{

   
    List<string> rewardNames = new List<string>();

    private void Awake()
    {
        float totalRewards = weightedRewards.Count;
        float currentWeight = 1.0f / totalRewards;
        int group = 0;
        foreach(WeightedRewardSource wr in weightedRewards) 
        {
            rewardNames.Add(wr.SourceName);
            DebugGUI.SetGraphProperties(wr.SourceName, wr.SourceName, 0.0f, 1.0f, group, new Color(0, currentWeight, 1), false);
            currentWeight += 1.0f / totalRewards;
            group ++;
        }


        // Set up graph properties using our graph keys
        // DebugGUI.SetGraphProperties("deepMimicReward", "Deep Mimic Reward", 0, 200, 2, new Color(0, 1, 1), false);
        //DebugGUI.SetGraphProperties("targetReward", "Target Reward", 0, 200, 2, new Color(1, 0.5f, 1), false);
     
    }


    private unsafe void Start()
    {
        if (MjScene.InstanceExists && (MjScene.Instance.Data != null))
        {

            OnAgentStart();

        }
        else {
            MjScene.Instance.sceneCreatedCallback += (_, _) => OnAgentStart();

        }
    }


    private void Update()
    {
        foreach (WeightedRewardSource wr in weightedRewards)
        {

   
           // DebugGUI.LogPersistent(wr.SourceName, wr.SourceName + " " + (wr.Reward).ToString("F3"));
            DebugGUI.Graph(wr.SourceName, wr.Reward);

        }


    }



    void OnDestroy()
    {

        foreach (string rn in rewardNames )
        {
            DebugGUI.RemoveGraph(rn);
        }


    }
}
