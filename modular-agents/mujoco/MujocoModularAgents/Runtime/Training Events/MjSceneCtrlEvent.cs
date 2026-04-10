using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using static Mujoco.MjScene;

public class MjSceneCtrlEvent : TrainingEvent
{
    public override void SubscribeHandler(EventHandler subscriber)
    {
        MjScene.Instance.ctrlCallback += new EventHandler<MjStepArgs>(subscriber);
    }

    public override void UnsubscribeHandler(EventHandler subscribed)
    {
        if(MjScene.InstanceExists) MjScene.Instance.ctrlCallback -= new EventHandler<MjStepArgs>(subscribed);
    }
}
