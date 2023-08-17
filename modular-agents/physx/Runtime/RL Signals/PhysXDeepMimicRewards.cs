
using System.Linq;
using ModularAgents.Kinematic.PhysX;

namespace ModularAgents.DeepMimic
{ 
public class PhysXDeepMimicRewards : DeepMimicRewards
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