using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents.Sensors;
using UnityEngine;

using ModularAgents.Kinematic;

namespace ModularAgents.DeepMimic
{
    /// <summary>
    /// An abstract ObservationSource that is completed for MuJoCo and PhysX in their respective ModularAgents packages.
    /// The physics engine specific initialization of the IKinematic adapter objects is left up for those child classes.
    /// e.g. see MjDeepMimicObservations in modular-agents/mujoco.
    /// </summary>
    public abstract class DeepMimicObservations : ObservationSource
    {

        [SerializeField]
        protected Transform rootTransform;

        [SerializeField]
        protected bool usePhase = true;
        protected IKinematic root;
        protected IReadOnlyList<IKinematic> observedKinematics;

     

        public override int Size => (FilterTransforms(rootTransform.GetComponentsInChildren<Transform>()).Count()) * 15 + (usePhase ? 1 : 0);

        public override void FeedObservationsToSensor(VectorSensor sensor)
        {
            ReferenceFrame fDyn = new ReferenceFrame(root.Forward, root.Position);

            
            

            foreach (IKinematic k in observedKinematics)
            {
                sensor.AddObservation(fDyn.WorldDirectionToCharacter(k.Velocity));
                sensor.AddObservation(fDyn.WorldDirectionToCharacter(k.AngularVelocity));       
                sensor.AddObservation(fDyn.WorldToCharacter(k.Position));
                (var normal, var tangent) = k.TransformMatrix.ToNormalTangent();
                sensor.AddObservation(fDyn.WorldDirectionToCharacter(normal));
                sensor.AddObservation(fDyn.WorldDirectionToCharacter(tangent));
            }
            if (usePhase)
            {
                sensor.AddObservation(GetPhase() % 1);
            }

        }

        protected abstract float GetPhase();

        protected abstract IEnumerable<Transform> FilterTransforms(IEnumerable<Transform> transformCollection);

		// Initializing the IKinematic objects is left for the physics engine specific child classes of DeepMimicObservations.
        public abstract override void OnAgentStart(); 
    }

}