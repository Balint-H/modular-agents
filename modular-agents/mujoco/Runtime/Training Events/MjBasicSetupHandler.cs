using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Mujoco;
using System.Data;
using ModularAgents.Kinematic;


namespace ModularAgents.TrainingEvents 
{ 
    public class MjBasicSetupHandler : BasicSetupHandler
    {
        protected override void SetupKineticChain()
        {
            kineticChainToReset = new ResettableMjBody(kineticRagdollRoot.GetComponentInChildren<MjBody>());
        }

        public unsafe override void HandleSetup(object sender, EventArgs eventArgs)
        {
            base.HandleSetup(sender, eventArgs);
            MujocoLib.mj_forward(MjScene.Instance.Model, MjScene.Instance.Data);
        }

        private class ResettableMjBody : IResettable
        {
            MjBody rootBody;
            public ResettableMjBody(MjBody rootBody)
            {
                this.rootBody = rootBody;
            }
            public void CopyKinematicsFrom(IKinematicReference reference, Vector3 offset)
            {
                MjKinematicRig mjReference = reference as MjKinematicRig;
                var sourceKinematics = MjState.GetMjKinematics(mjReference.KinematicRagdollRoot.GetComponent<MjBody>());
            

                if (offset != Vector3.zero)
                {
                    
                    var rootKinematics = MjState.GetRootKinematics(mjReference.KinematicRagdollRoot.GetComponent<MjBody>());
                    var mjOffset =
                       (offset);

                    var rootPos = rootKinematics.Item1;
                    rootPos[0] += mjOffset[0];
                    rootPos[1] += mjOffset[1];
                    rootPos[2] += mjOffset[2];

                    int index = reference.RagdollTransforms.ToList().FindIndex(x => x.Equals(mjReference.KinematicRagdollRoot));

                    var sourcePositions = sourceKinematics.Item1.ToArray();
                    sourcePositions[index] = rootPos;

                   sourceKinematics.Item1 = sourcePositions;

                }

                MjScene.Instance.SetMjKinematics(rootBody, sourceKinematics.Item1, sourceKinematics.Item2);
            }

            public void TeleportRoot(Vector3 position, Quaternion rotation)
            {
                MjState.TeleportMjRoot(rootBody.GetComponentInChildren<MjFreeJoint>(), position, rotation);
               
            }
        }
    }


}