using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Mujoco;
using System.Linq;
using ModularAgentsRecorder;

using ModularAgents.Kinematic;
#region To convert this RL signal to PhysX, this region needs to be replaced with `using Kinematic`
using ModularAgents.Kinematic.Mujoco;
#endregion

namespace ModularAgents.DeepMimic
{

    public class MjDeepMimicRewards : DeepMimicRewards
    {
        public override void OnAgentStart()
        {
          
            simChain = simRoot.GetBodyChain();
            kinChain = kinRoot.GetBodyChain();


            simEEs = simEETransforms.Select(x => x.GetIKinematic()).ToList();
            kinEEs = kinEETransforms.Select(x => x.GetIKinematic()).ToList();


        }



        public void LogRewards(ValueRecorder recorder)
        {
            ReferenceFrame simFrame = new ReferenceFrame(simChain.RootForward, simChain.Root.Position); // Not CoM as then the CoM reward would have no meaning
            ReferenceFrame kinFrame = new ReferenceFrame(kinChain.RootForward, kinChain.Root.Position); // Not CoM as then the CoM reward would have no meaning

            /*
            public float CalculateRewards()
            {
                ReferenceFrame simFrame = new ReferenceFrame(simChain.RootForward, simChain.Root.Position); // Not CoM as then the CoM reward would have no meaning
                ReferenceFrame kinFrame = new ReferenceFrame(kinChain.RootForward, kinChain.Root.Position); // Not CoM as then the CoM reward would have no meaning


                return 0.65f * PoseReward(kinChain, simChain) +
                        0.1f * VelocityReward(kinChain, simChain) +
                        0.15f * EndEffectorReward(kinEEs, simEEs, kinFrame, simFrame, useGlobalPositions) +
                        0.1f * CenterOfMassReward(kinChain, simChain, kinFrame, simFrame, useGlobalPositions);
            }
             */


            double pr =  PoseReward(kinChain, simChain) ;
            double vr = VelocityReward(kinChain, simChain);
            double er = EndEffectorReward(kinEEs, simEEs, kinFrame, simFrame, useGlobalPositions);
            double cr = CenterOfMassReward(kinChain, simChain, kinFrame, simFrame, useGlobalPositions);
            recorder.Record( new double[1] { pr },"reward_pose_" + name );
            recorder.Record(new double[1] { vr }, "reward_velocity_" + name);
            recorder.Record(new double[1] { er }, "reward_ee_" + name);
            recorder.Record(new double[1] {cr }, "reward_com_" + name);

            recorder.Record(new double[1] {
                        0.65f * pr +
                        0.1f * vr +
                        0.15f * er +
                        0.1f * cr          }, "reward_global_" + name);

        }



    }

}