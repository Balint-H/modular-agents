using ModularAgents.Kinematic.Mujoco;
using ModularAgents.MotorControl;
using Mujoco.Extensions;

namespace ModularAgents.DReCon
{
    public class MjDReConObservationSource : DReConObservationSource
    {

        public override void OnAgentStart()
        {
            userInputs = inputObject.GetComponent<IAnimationController>();
            previousActionProvider = previousActionGameObject.GetComponent<IRememberPreviousActions>();

            MjState.ExecuteAfterMjStart(MjInitialize);
            
        }

        private void MjInitialize()
        {
            kinChain = kinematicTransform.GetBodyChain();
            simChain = simulationTransform.GetBodyChain();
            kinSubsetBodies = kinematicSubset.GetBodyChain();
            simSubsetBodies = simulationSubset.GetBodyChain();
        }
    }
}