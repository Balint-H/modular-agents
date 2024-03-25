using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;

public class MjScenePreEvent : TrainingEvent
{

    public override void SubscribeHandler(EventHandler subscriber)
    {
        MjScene.Instance.preUpdateEvent += (sender, args) => subscriber(sender, args);  // TODO: Confirm this works as intended
    }

    public override void UnsubscribeHandler(EventHandler subscribed)
    {
        if (MjScene.InstanceExists) MjScene.Instance.preUpdateEvent -= (sender, args) => subscribed(sender, args);
    }
}