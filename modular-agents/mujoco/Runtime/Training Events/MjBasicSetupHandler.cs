using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Mujoco;
using Mujoco.Extensions;
using System.Data;
using ModularAgents.Kinematic;
using ModularAgents.Kinematic.Mujoco;

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
            public void CopyKinematicsFrom(IKinematicReference reference, Vector3 offset, Transform rootRef = null)
            {
                MjKinematicRig mjReference = reference as MjKinematicRig;

                (IEnumerable<double[]>, IEnumerable<double[]>) sourceKinematics;
                (double[], double[]) rootKinematics;


                if (mjReference != null)
                {
              

              
                    MjBody kinRagRoot = mjReference.KinematicRagdollRoot.GetComponent<MjBody>();
                    sourceKinematics = MjState.GetMjKinematics(kinRagRoot);
                    // rootKinematics = MjState.GetRootKinematics(mjReference.KinematicRagdollRoot.GetComponent<MjBody>());
                    rootKinematics = MjState.GetRootKinematics(kinRagRoot);



                    if (offset != Vector3.zero)
                    {


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

                /*
                else //we are in a finite difference case:
                
                {
                    (IEnumerable<double[]>, IEnumerable<double[]>) sourceKinematicsFD;
                    (double[], double[]) rootKinematicsFD;


              
                    MjKinematicRigFD mjReferenceFD = reference as MjKinematicRigFD;
                    //MjKinematicRigFD mjReferenceFD = rootRef.GetComponent<MjKinematicRigFD>();
                    sourceKinematicsFD = mjReferenceFD.GetMjKinematics();
                    rootKinematicsFD = mjReferenceFD.GetRootKinematics();

                 

                    MjScene.Instance.SetMjKinematics(rootBody, sourceKinematicsFD.Item1, sourceKinematicsFD.Item2);

                }
                */


                //MjScene.Instance.SetMjKinematics(rootBody, sourceKinematics.Item1, sourceKinematics.Item2);


            }

            public void TeleportRoot(Vector3 position, Quaternion rotation)
            {
                MjState.TeleportMjRoot(rootBody.GetComponentInChildren<MjFreeJoint>(), position, rotation);
               
            }
        }
    }


}