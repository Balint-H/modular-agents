using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Mujoco;
using System.Linq;


#region To convert this RL signal to PhysX, this region needs to be replaced with `using Kinematic`
using ModularAgents.Kinematic.Mujoco;
#endregion

namespace ModularAgents.DeepMimic
{

    public class MjDeepMimicRewards : DeepMimicRewards
    {
        public override void OnAgentStart()
        {
            kinChain = kinRoot.GetBodyChain();
            simChain = simRoot.GetBodyChain();
            kinEEs = kinEETransforms.Select(x => x.GetIKinematic()).ToList();
            simEEs = simEETransforms.Select(x => x.GetIKinematic()).ToList();
        }
    }

}