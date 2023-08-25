using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;

namespace ModularAgents.TrainingEvents
{

    public class MjSynchroniseHandler : TrainingEventHandler
    {
        public override EventHandler Handler => (_,_) => MjScene.Instance.SyncUnityToMjState();
    }
}