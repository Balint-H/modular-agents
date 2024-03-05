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




        public void LogObservations(ValueRecorder recorder)
        {

            if (recorder != null) 
            {
                ReferenceFrame fDyn = new ReferenceFrame(root.Forward, root.Position);

            

                foreach (IKinematic k in observedKinematics)
                {
                   

                    recorder.Record(fDyn.WorldDirectionToCharacter(k.Velocity), name + "_" + k.Name+ "_velocity");

                    recorder.Record(fDyn.WorldDirectionToCharacter(k.AngularVelocity), name + "_" + k.Name + "_angularVelocity");
                    recorder.Record(fDyn.WorldToCharacter(k.Position), name + "_" +  k.Name + "_position");


                    (var normal, var tangent) = k.TransformMatrix.ToNormalTangent();
                    
                    recorder.Record(fDyn.WorldDirectionToCharacter(normal), name + "_" + k.Name + "_normal");
                    recorder.Record(fDyn.WorldDirectionToCharacter(tangent), name + "_" + k.Name + "_tangent");

                }
                if (usePhase)
                {
                    float[] p = new float[1] { GetPhase() % 1 };
                    recorder.Record(p, name + "_" + "anim_phase");
                    
                }



            }


        
        }




        public void LogObservationsSimpler(ValueRecorder recorder)
        {

            if (recorder != null)
            {


                foreach (IKinematic k in observedKinematics)
                {
                    recorder.Record(k.Velocity, "simpler_" +  name + "_" + k.Name + "_velocity");
                    recorder.Record(k.AngularVelocity, "simpler_" + name + "_" + k.Name + "_angularVelocity");
                    recorder.Record(k.Position, "simpler_" +  name + "_" + k.Name + "_position");
                    recorder.Record(k.Rotation, "simpler_" +  name + "_" + k.Name + "_rotation");
                    recorder.Record(k.LocalRotation, "simpler_" + name + "_" + k.Name + "_localRotation");

                }
                if (usePhase)
                {
                    float[] p = new float[1] { GetPhase() % 1 };
                    recorder.Record(p, "simpler_" + name + "_" + "anim_phase");

                }



            }



        }





    }

}