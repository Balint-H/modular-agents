using ModularAgents.Kinematic.Mujoco;
using ModularAgents.MotorControl;


namespace ModularAgents.DReCon
{
    public class MjDReConObservationSource : DReConObservationSource
    {

        public override void OnAgentStart()
        {
            userInputs = inputObject.GetComponent<IAnimationController>();
            previousActionProvider = previousActionGameObject.GetComponent<IRememberPreviousActions>();

            kinChain = kinematicTransform.GetBodyChain();
            simChain = simulationTransform.GetBodyChain();
            kinSubsetBodies = kinematicSubset.GetBodyChain();
            simSubsetBodies = simulationSubset.GetBodyChain();
        }
    }
}