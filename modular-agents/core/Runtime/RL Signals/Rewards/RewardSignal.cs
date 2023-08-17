using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using Unity.MLAgents;
using UnityEngine;

namespace ModularAgents
{
    /// <summary>
    /// Collects rewards from your custom RewardSource behaviours, and provides their mixed rewards to the ModularAgent.
    /// RewardSignal is a TrainingEventHandler because it can be used for sparse rewards as well. 
    /// If it's paired with TrainingEvents, it will only calculate and assign a reward when its invoked by its triggering events.
    /// </summary>
    public class RewardSignal : TrainingEventHandler
    {
        [SerializeField]
        protected List<WeightedRewardSource> weightedRewards;
        protected List<WeightedRewardSource> nonEmptyRewards;

        [SerializeField]
        [DefaultValue(RewardMixing.MixType.Linear)]
        private RewardMixing.MixType MixingMethod;

        bool IsSparse =>triggeringEvents != null && triggeringEvents.Count != 0;

        ModularAgent modularAgent;

        public event Action<float> OnCalculateReward;

        public float Reward
        {
            get
            {
                var reward = nonEmptyRewards.MixRewards(MixingMethod);
                OnCalculateReward?.Invoke(reward);
                return reward;
            }
        }

        public override EventHandler Handler => (_, _) => modularAgent.AddReward(Reward);

        

        public void OnAgentStart(ModularAgent modularAgent=null)
        {
            this.modularAgent = modularAgent;
            nonEmptyRewards = weightedRewards.Where(r => !r.IsEmpty()).ToList();
            foreach (var wr in nonEmptyRewards)
            {
                wr.OnAgentStart();
            }

            if (!IsSparse && modularAgent)
            {
                modularAgent.OnAfterAction += Handler;
            }
        }


        // TODO: Implement with custom editor instead.
        private void OnValidate()
        {
            if (!IsSparse && triggeringEvents != null && triggeringEvents.Count > 0)
            {
                Debug.LogWarning("Non-sparse rewards cannot not have triggering events! They are assigned after the agent's decisions automatically.");
                triggeringEvents.Clear();
            }
        }

    }

    public static class RewardMixing
    {
        public enum MixType
        {
            Linear,
            Unweighted,
            Multiplicative
        }

        /// <summary>Weighted sum of a collection reward sources</summary>
        public static float LinearMix(this IEnumerable<WeightedRewardSource> rewardList)
        {
            return rewardList.Select(x => x.Weight * x.Reward).Sum();
        }

        /// <summary>Sum of a collection reward sources, ignoring the weight field in the WeightedRewardSource</summary>
        public static float UnweightedMix(this IEnumerable<WeightedRewardSource> rewardList)
        {
            return rewardList.Select(x => x.Reward).Sum();
        }

        public static float MultiplicativeMix(this IEnumerable<WeightedRewardSource> rewardList)
        {
            return rewardList.Select(x => x.Reward).Product();
        }

        public static float Product(this IEnumerable<float> nums)
        {
            return nums.Aggregate(1f, (acc, val) => acc * val);
        }

        ///<summary>Mix rewards with method selected by enum</summary>
        public static float MixRewards(this IEnumerable<WeightedRewardSource> rewardsToMix, MixType mixType)
        {
            switch (mixType)
            {
                case MixType.Linear:
                    return rewardsToMix.LinearMix();

                case MixType.Unweighted:
                    return rewardsToMix.UnweightedMix();

                case MixType.Multiplicative:
                    return rewardsToMix.MultiplicativeMix();

                default:
                    return rewardsToMix.LinearMix();
            }
        }
    }
}