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
        MjFreeJoint freeJointToTeleport = null;

        [SerializeField]
        Animator  animatorToTeleport = null;



        [SerializeField]
        Transform destination;

        [SerializeField]
        float freeJointHeight;

        [SerializeField]
        bool shouldZeroVelocityAndAcceleration;

        public override EventHandler Handler => (_, _) => Teleport();

        private unsafe void Teleport()
        {
            if (freeJointToTeleport != null)
            {

                MjState.TeleportMjRoot(freeJointToTeleport, destination.position + new Vector3(0,freeJointHeight,0), destination.rotation);
            }
            
            if(animatorToTeleport != null)
                animatorToTeleport.transform.position = destination.position;

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