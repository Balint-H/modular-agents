using Mujoco;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents.Sensors;
using UnityEngine;

using ModularAgents.Kinematic.Mujoco;

using ModularAgents.Kinematic;
using ModularAgentsRecorder;

namespace ModularAgents.DeepMimic
{
    public class MjDeepMimicObservations : DeepMimicObservations
    {
        [SerializeField]
        protected Animator animator;

        [SerializeField]
        ValueRecorder recorder;


        protected override IEnumerable<Transform> FilterTransforms(IEnumerable<Transform> transformCollection)
        {
            return transformCollection
                .Where(t => t.IsIKinematic())
                .Where(mjb => mjb.GetComponentInDirectChildren<MjBaseJoint>() && !mjb.GetComponentInDirectChildren<MjFreeJoint>())
                .Where(t => !t.name.Contains("toe", System.StringComparison.OrdinalIgnoreCase) && !t.name.Contains("neck", System.StringComparison.OrdinalIgnoreCase))
                .Select(mjb => mjb.transform);
        }

        public override void OnAgentStart()
        {
            root = rootTransform.GetIKinematic();
            observedKinematics = FilterTransforms(rootTransform.GetComponentsInChildren<Transform>()).Select(x => x.GetIKinematic() ).ToList();
        }

        protected override float GetPhase()
        {
            return animator.GetCurrentAnimatorStateInfo(0).normalizedTime;
        }




        public override void FeedObservationsToSensor(VectorSensor sensor)
        {

            if (recorder != null) 
            {
                ReferenceFrame fDyn = new ReferenceFrame(root.Forward, root.Position);

                foreach (IKinematic k in observedKinematics)
                {
                   

                    recorder.Record(fDyn.WorldDirectionToCharacter(k.Velocity), k.Name+ "_velocity");

                    recorder.Record(fDyn.WorldDirectionToCharacter(k.AngularVelocity), k.Name + "_angularVelocity");
                    recorder.Record(fDyn.WorldToCharacter(k.Position), k.Name + "_position");


                    (var normal, var tangent) = k.TransformMatrix.ToNormalTangent();
                    
                    recorder.Record(fDyn.WorldDirectionToCharacter(normal), k.Name + "_normal");
                    recorder.Record(fDyn.WorldDirectionToCharacter(tangent), k.Name + "_tangent");

                }
                if (usePhase)
                {
                    float[] p = new float[1] { GetPhase() % 1 };
                    recorder.Record(p,  "anim_phase");
                    
                }






            }


            base.FeedObservationsToSensor(sensor);

            /*
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
            */
        }




    }

}