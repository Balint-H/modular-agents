using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

public class RewardCounter : TrainingEventHandler
{
    [SerializeField]
    Agent agent;

    IEventsAgent eventsAgent;
    public override EventHandler Handler => PrintEvents;

    private float rewardSum;
    private float averageReward;
    private int numberOfSteps;

    private float episodeSum;
    private int numEpisodes;

    private void Awake()
    {
        eventsAgent = agent as IEventsAgent;
        rewardSum = 0;
        averageReward = 0;
        numberOfSteps = 0;
        eventsAgent.OnPostAction += AccumulateRewards;

    }

    private void FixedUpdate()
    {
        Debug.Log(agent.GetCumulativeReward());
    }

    private void AccumulateRewards(object sender, AgentEventArgs args)
    {
        rewardSum += args.reward;
        numberOfSteps++;

        averageReward = rewardSum / numberOfSteps;
    }

    public void PrintEvents(object sender, EventArgs args)
    {
        numEpisodes++;

        Debug.Log($"Total reward: {rewardSum}\nAverage reward per episode: {episodeSum / numEpisodes}\nAverage reward per action: {averageReward}");

        episodeSum += rewardSum;

        rewardSum = 0;
        numberOfSteps = 0;
        averageReward = 0;

        
        



    }
}
