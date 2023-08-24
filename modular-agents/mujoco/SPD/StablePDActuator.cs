using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;
using ModularAgents.MotorControl;
using Mujoco;
using Mujoco.Extensions;
using MathNet.Numerics.LinearAlgebra;
using static ModularAgents.MathNet.Numerics.LinearAlgebra.LinAlgUtils;

namespace ModularAgents.DReCon
{
    /// <summary>
    /// Deprecated, as specific to a non-generic agent type. Will be removed in a future release. Use SPDActuatorComponent instead.
    /// </summary>
    public class StablePDActuator : DReConActuator
    {
        [SerializeField]
        protected Transform root;

        [SerializeField, Tooltip("No action assigned to these joints, copy from reference if available")]
        protected List<MjBaseJoint> softExcludeList;

        [SerializeField, Tooltip("No action assigned to these joints")]
        protected List<MjBaseJoint> hardExcludeList;

        [SerializeField]
        double posGain;

        [SerializeField]
        double velGain;

        [SerializeField]
        double maxForce;

        protected IReadOnlyList<IMjJointState> jointStates;
        protected IReadOnlyList<IMjJointState> activeReferenceStates;
        protected int[] dofAddresses;
        protected int[] activeDofLocalIndices; // Maps actions of agent to the array of DoFs actuated by this component
        protected Dictionary<(int, int), int> inertiaSubMatrixMap;

        protected Matrix<double> posGainMatrix;
        protected Matrix<double> velGainMatrix;
        public virtual IEnumerable<MjBaseJoint> Joints { get => root.GetComponentsInChildren<MjBaseJoint>().Where(j => j is not MjFreeJoint); }
        public virtual IEnumerable<MjBaseJoint> ActiveJoints
        {
            get => IsExcludeDefined ? Joints.Where(j => !softExcludeList.Contains(j) && !hardExcludeList.Contains(j))
                                    : Joints;
        }

        public IReadOnlyList<IJointState> JointStates => jointStates;

        [SerializeField]
        Transform kinematicRef;

        [SerializeField, Range(0.0f, 1.0f)]
        float compensation;

        [SerializeField]
        bool trackState;

        [SerializeField]
        bool updateAlone;

        [SerializeField]
        bool updateGains;

        [SerializeField]
        bool useRegularPD;

        [SerializeField, Range(0, 5)]
        double actionScale = 1;

        double dt;

        private bool IsExcludeDefined { get => (softExcludeList != null && hardExcludeList != null && softExcludeList.Count + hardExcludeList.Count > 0); }
        private int ExcludedDofCount => IsExcludeDefined ? softExcludeList.DofSum() + hardExcludeList.DofSum() : 0;

        public override int ActionSpaceSize => ActiveJoints.DofSum();

        Vector<double> nextActions;

        public Vector<double> PosError => useRegularPD ? IMjJointState.GetPosErrorVector(jointStates) + nextActions : IMjJointState.GetStablePosErrorVector(jointStates, dt) + nextActions;
        public Vector<double> VelError => IMjJointState.GetVelErrorVector(jointStates);
        unsafe private void UpdateTorque(object sender, MjStepArgs e)
        {
            var posError = useRegularPD? IMjJointState.GetPosErrorVector(jointStates) + nextActions : IMjJointState.GetStablePosErrorVector(jointStates, dt) + nextActions;
            var velError = IMjJointState.GetVelErrorVector(jointStates);

            if (updateGains)
            {
                posGainMatrix = Matrix<double>.Build.Diagonal(dofAddresses.Length, dofAddresses.Length, posGain);
                velGainMatrix = Matrix<double>.Build.Diagonal(dofAddresses.Length, dofAddresses.Length, velGain);
            }

            Vector<double> biasVector = (Vector<double>.Build.DenseOfArray(MjState.GetSubBias(dofAddresses, e)) + Vector<double>.Build.DenseOfArray(MjState.GetSubPassive(dofAddresses, e)));
            Matrix<double> inertiaMatrix = MjState.GetSubInertiaArray(inertiaSubMatrixMap, dofAddresses.Length, e).ToSquareMatrix(dofAddresses.Length);

            var generalizedForces = ComputeSPD(posError, velError, posGainMatrix, velGainMatrix, (1 - compensation) * biasVector, inertiaMatrix, useRegularPD? 0: dt) + biasVector * compensation;
            
            foreach ((var dofIdx, var force) in dofAddresses.Zip(generalizedForces, Tuple.Create))
            {
                if (double.IsNaN(force))
                {
                    Debug.Log("Nan!");
                }
                e.data->qfrc_applied[dofIdx] = Math.Clamp(force, -maxForce, maxForce);
            }
        }

        unsafe public override void ApplyActions(float[] actions, float actionTimeDelta)
        {
            nextActions = actionScale * ActionsToVector(actions, activeDofLocalIndices, dofAddresses.Length);
        }

        public override float[] GetActionsFromState()
        {
            return (kinematicRef && !trackState) ? activeReferenceStates.SelectMany(rs => rs.PositionErrors).Select(p => -(float)(p / actionScale)).ToArray() : Enumerable.Repeat(0f, ActionSpaceSize).ToArray();
        }

        private static Vector<double> ActionsToVector(float[] actions, int[] indices, int dofCount)
        {
            double[] castExpandedActions = new double[dofCount];
            for (int i = 0; i < actions.Length; i++)
            {
                castExpandedActions[indices[i]] = actions[i];
            }

            return Vector<double>.Build.DenseOfArray(castExpandedActions);
        }

        public void SetGains(double kp, double kd)
        {
            posGain = kp;
            velGain = kd;
            if (!updateGains) Debug.LogWarning("Static gains, so changing values won't have effect");
        }

        public unsafe override void OnAgentInitialize(DReConAgent agent)
        {
            MjScene.Instance.ctrlCallback += UpdateTorque;
            var actuatedJoints = hardExcludeList != null ? Joints.Where(j => !hardExcludeList.Contains(j)) : Joints;
            var activeJoints = ActiveJoints;

            Func<MjBaseJoint, bool> IsActive = (MjBaseJoint j) => !IsExcludeDefined || activeJoints.Contains(j);
            Func<IEnumerable<MjBaseJoint>, IEnumerable<bool>> GetDofActivity = (IEnumerable<MjBaseJoint> js) => js.Select(j => Enumerable.Repeat(IsActive(j), j.DofCount())).SelectMany(x => x);
            Func<IEnumerable<bool>, IEnumerable<int>> GetIndicesOfTrue = (IEnumerable<bool> bs) => bs.Select((b, i) => (b, i)).Where(enumerated => enumerated.b).Select(enumerated => enumerated.i);
            Func<IEnumerable<MjBaseJoint>, IEnumerable<int>> GetActiveDofIndices = (IEnumerable<MjBaseJoint> js) => GetIndicesOfTrue(GetDofActivity(js));

            if (kinematicRef && trackState)
            {
                jointStates = actuatedJoints.Select(j => IMjJointState.GetJointState(j, FindReference(j))).ToList();
                activeReferenceStates = jointStates.Where(js => IsActive(js.Joint)).Select(js => js.ReferenceState).ToList();
            }
            else
            {
                jointStates = actuatedJoints.Select(j => IMjJointState.GetJointState(j)).ToList();

                activeReferenceStates = jointStates.Where(js => IsActive(js.Joint)).Select(js => IMjJointState.GetJointState(FindReference(js.Joint))).ToList();
            }

            activeDofLocalIndices = GetActiveDofIndices(actuatedJoints).ToArray();
            dofAddresses = IMjJointState.GetDofAddresses(jointStates);

            nextActions = ActionsToVector(GetActionsFromState(), activeDofLocalIndices, dofAddresses.Length);
            
            posGainMatrix = Matrix<double>.Build.Diagonal(dofAddresses.Length, dofAddresses.Length, posGain);
            velGainMatrix = Matrix<double>.Build.Diagonal(dofAddresses.Length, dofAddresses.Length, velGain);

            inertiaSubMatrixMap = MjState.GetInertiaSubMatrixIndexMap(dofAddresses.ToList(), new MjStepArgs(MjScene.Instance.Model, MjScene.Instance.Data));
            dt = Time.fixedDeltaTime;
        }

        private void OnDisable()
        {
            if (MjScene.InstanceExists) MjScene.Instance.ctrlCallback -= UpdateTorque;
        }

        private MjBaseJoint FindReference(MjBaseJoint joint)
        {
            return kinematicRef ? kinematicRef.GetComponentsInChildren<MjBaseJoint>().First(rj => rj.name.Contains(joint.name)) : null;
        }

        private void Start()
        {
            if (updateAlone)
            {
                OnAgentInitialize(null);
            }
        }

        private void FixedUpdate()
        {
            
        }

        private void Update()
        {
            
        }

        /// <summary>
        /// Calculate Stable PD forces as per Tan et al.
        /// </summary>
        /// <param name="posErrors">Should include first order Taylor expansion</param>
        /// <param name="velErrors"></param>
        /// <param name="biasForces">Should include passive joint forces as well</param>
        /// <param name="dt"></param>
        /// <returns></returns>
        private static Vector<double> ComputeSPD(Vector<double> posErrors, Vector<double> velErrors, Matrix<double> Kp, Matrix<double> Kd, Vector<double> biasForces, Matrix<double> M, double dt)
        {
            var pTerm = Kp * posErrors;
            var dTerm = Kd * velErrors;
            var spdQacc = (M + Kd * dt).Solve(pTerm + dTerm - biasForces);
            var tau = pTerm + dTerm - Kd * spdQacc * dt;
            return tau;
        }


    }
}
