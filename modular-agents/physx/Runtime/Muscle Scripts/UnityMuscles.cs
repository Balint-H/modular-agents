using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using ModularAgents.DReCon;


namespace ModularAgents.MotorControl.PhysX
{
    public abstract class UnityMuscles : DReConActuator
    {

        protected List<ArticulationBody> _motors;
        public bool UseStateTracking;
        public Transform kinematicRef;

        [SerializeField]
        protected PhysXActuatorReferencePair[] trackedRefs;

        public override int ActionSpaceSize { get => CalculateActionSpaceSize(); }



        int CalculateActionSpaceSize()
        {

            List<ArticulationBody> motors = GetMotors();

            /*
                string s = "";

                foreach (ArticulationBody ab in motors)
                    s = s + ab.name + " " + ab.dofCount + "\n";

                Debug.Log(s);
            */
            return motors.Select(x => x.dofCount).Sum();


        }



        public override void OnAgentInitialize(DReConAgent agent = null)
        {
            InitMotors();
        }

        protected void InitMotors()
        {
            _motors = GetMotors();
            if (UseStateTracking)
                InitTrackedReferences();
        }

        protected void InitTrackedReferences()
        {
            trackedRefs = PhysXActuatorReferencePair.GetActuatorReferencePairs(_motors.ToArray(), GetReferences());

        }



        public virtual List<ArticulationBody> GetMotors()
        {
            List<ArticulationBody> motors = GetComponentsInChildren<ArticulationBody>()
                    .Where(x => x.jointType == ArticulationJointType.SphericalJoint)
                    .Where(x => !x.isRoot)
                    .Distinct()
                    .ToList();
            return motors;
        }



        public override float[] GetActionsFromState()
        {
            if (UseStateTracking)
                return (PhysXActuatorReferencePair.GetTargetPositions(trackedRefs));
            else
                return Enumerable.Repeat(0f, ActionSpaceSize).ToArray();
        }

        public Rigidbody[] GetReferences()
        {
            List<Rigidbody> references = new List<Rigidbody>();
            foreach (ArticulationBody ab in _motors)
                references.Add(FindReference(ab));

            return references.ToArray();
        }

        private Rigidbody FindReference(ArticulationBody joint)
        {
            return kinematicRef.GetComponentsInChildren<Rigidbody>().First(rj => rj.name.Contains(joint.name));
        }
    }
}
