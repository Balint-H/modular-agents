using Mujoco;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco.Extensions;
namespace ModularAgents.TrainingEvents
{

    public class MjTeleportHandler : TrainingEventHandler
    {
        [SerializeField]
        MjFreeJoint freeJointToTeleport;

        [SerializeField]
        Transform destination;

        [SerializeField]
        bool shouldZeroVelocityAndAcceleration;

        public override EventHandler Handler => (_, _) => Teleport();

        private unsafe void Teleport()
        {
            MjState.TeleportMjRoot(freeJointToTeleport, destination.position, destination.rotation);
            if(shouldZeroVelocityAndAcceleration )
            {
                MjEngineTool.SetMjVector3(MjScene.Instance.Data->qacc + freeJointToTeleport.DofAddress, Vector3.zero);
                MjEngineTool.SetMjVector3(MjScene.Instance.Data->qacc + freeJointToTeleport.DofAddress + 3, Vector3.zero);
                MjEngineTool.SetMjVector3(MjScene.Instance.Data->qacc + freeJointToTeleport.DofAddress, Vector3.zero);
                MjEngineTool.SetMjVector3(MjScene.Instance.Data->qacc + freeJointToTeleport.DofAddress + 3, Vector3.zero);
            }
        }

    }
}