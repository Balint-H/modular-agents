using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;

public class MjScenePostEvent : TrainingEvent
{
    public override void SubscribeHandler(EventHandler subscriber)
    {
        MjScene.Instance.postUpdateEvent += (sender, args) => subscriber(sender, args);  // TODO: Confirm this works as intended
    }

    public override void UnsubscribeHandler(EventHandler subscribed)
    {
        if (MjScene.InstanceExists) MjScene.Instance.postUpdateEvent -= (sender, args) => subscribed(sender, args);
    }
}
