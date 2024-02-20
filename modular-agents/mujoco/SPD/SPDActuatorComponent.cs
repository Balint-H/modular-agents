using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;
using MathNet.Numerics.LinearAlgebra;
using Unity.MLAgents.Actuators;
using Mujoco;
using Mujoco.Extensions;
using static ModularAgents.MathNet.Numerics.LinearAlgebra.LinAlgUtils;

namespace ModularAgents.MotorControl.Mujoco
{
    /// <summary>
    /// Stable PD controller. Actions interpreted as positions only. Optionally uses a reference state as a baseline (the trackState param).
    /// Reference:
    /// Tan, J., Liu, K. and Turk, G., 2011. Stable proportional-derivative controllers. IEEE Computer Graphics and Applications, 31(4), pp.34-44.
    /// </summary>
    public class SPDActuatorComponent : ActuatorComponent
    {
        [SerializeField]
        protected Transform root;  // The root body of the controlled humanoid, all joints should be the child of this Transform.

        [SerializeField, Tooltip("No action assigned to these joints, copy from reference if available")]
        protected List<MjBaseJoint> softExcludeList;

        [SerializeField, Tooltip("No action assigned to these joints")]
        protected List<MjBaseJoint> hardExcludeList;

        [SerializeField]
        double posGain;  // The P part of PD

        [SerializeField]
        double velGain;  // The D part of PD

        [SerializeField]
        double maxForce;  // Useful for reducing instabilities, and forces more reasonable solutions from the Agent.

        // JointStates can calculate positions, velocities and its errors, used to calculate PD corrective forces. These include both controlled and "softExcluded" joints.
        protected IReadOnlyList<IMjJointState> jointStates;

        // The reference JointStates of only the actively controlled joints. Used for initializing at the start of an episode by copying the state of these joints to 
        // an IRememberPreviousActions object (e.g. so action smoothing is cleared from the previous episode)
        // TODO: replace this explicit reference with an event that all interested components may subscribe to.
        protected IReadOnlyList<IMjJointState> activeReferenceStates;

        public IReadOnlyList<IMjJointState> ActiveReferenceStates => activeReferenceStates;

        IRememberPreviousActions prevActionSource;

        // Addresses in the qfrc_applied array of MuJoCo controlled (either actively modulating by the agent, or "softExcluded" ones simply tracking)
        private int[] dofAddresses;
        protected int[] activeDofLocalIndices; // The array of qfrc indices corresponding to each action of the Agent. Could be in arbitrary order.

        // For a given index in the 2d inertia matrix consistent with dofAddresses, gives the index in the flattened, optimized inertia vector in mujoco.
        // For more infor see https://mujoco.readthedocs.io/en/latest/computation.html#equation-eq-motion
        protected Dictionary<(int, int), int> inertiaSubMatrixMap;  

        // Diagonal gain matrices
        private Matrix<double> posGainMatrix;
        private Matrix<double> velGainMatrix;

        public virtual IEnumerable<MjBaseJoint> Joints { get => root.GetComponentsInChildren<MjBaseJoint>().Where(j => j is not MjFreeJoint); }
        public virtual IEnumerable<MjBaseJoint> ControlledJoints
        {
            get => IsExcludeDefined ? Joints.Where(j => !softExcludeList.Contains(j) && !hardExcludeList.Contains(j))
                                    : Joints;
        }

        // Inclusive of "softExcluded" joints
        public virtual IEnumerable<MjBaseJoint> ActuatedJoints => hardExcludeList != null ? Joints.Where(j => !hardExcludeList.Contains(j)) : Joints;

        public IReadOnlyList<IMjJointState> JointStates => jointStates;

        [SerializeField]
        Transform kinematicRef;

        [SerializeField, Range(0.0f, 1.0f), Tooltip("How much gravity, coriolis and passive forces are compensated automatically (1 is completely).")]
        float compensation; 

        [SerializeField, Tooltip("If enabled, controller will follow along the reference automatically and actions will only modulate this.")]
        bool trackState;

        [SerializeField, Tooltip("If used without an Agent, enable to still perform PD Control with actions from Heuristic")]
        bool updateAlone;

        [SerializeField, Tooltip("Enable this if changes to the gain fields should be reflected during runtime.")]
        bool updateGains;

        [SerializeField, Tooltip("Skips the first order expansion approximations. Usually less stable, but may be necessary with some integrators.")]
        bool useRegularPD;  // For more info on integrators, see https://mujoco.readthedocs.io/en/latest/computation.html#numerical-integration

        [SerializeField, Tooltip("Supply baseline actions, useful if no Agent is used.")]
        bool useHeuristic;

        [SerializeField, Range(0, 5), Tooltip("Multiplies the action vector. Usually around 2 is good if trackState is false, and 1 otherwise.")]
        double actionScale = 1;

        [SerializeField, Tooltip("A GameObject that has an IRememberPreviousActions component that needs to be reset at the start of each episode (e.g. a SmoothedActuatorComponent).")]
        GameObject smoothingObject;

        [SerializeField]
        ModularAgent agent;

        double dt;

        private bool IsExcludeDefined { get => (softExcludeList != null && hardExcludeList != null && softExcludeList.Count + hardExcludeList.Count > 0); }
        private int ExcludedDofCount => IsExcludeDefined ? softExcludeList.DofSum() + hardExcludeList.DofSum() : 0;

        public int ActionSpaceSize => ControlledJoints.DofSum();

        Vector<double> nextActions;

        public Vector<double> PosError => useRegularPD ? IMjJointState.GetPosErrorVector(jointStates) + nextActions : IMjJointState.GetStablePosErrorVector(jointStates, dt) + nextActions;
        public Vector<double> VelError => IMjJointState.GetVelErrorVector(jointStates);

        public override ActionSpec ActionSpec => new ActionSpec(ActionSpaceSize);

        public Matrix<double> PosGainMatrix { get => posGainMatrix; set => posGainMatrix = value; }
        public Matrix<double> VelGainMatrix { get => velGainMatrix; set => velGainMatrix = value; }
        public IEnumerable<int> DofAddresses { get => dofAddresses;}
        public double PosGain { get => posGain;}
        public double VelGain { get => velGain;}

        unsafe private void UpdateTorque(object sender, MjStepArgs e)
        {
            //Debug.Log("Control");
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

        unsafe public void ApplyActions(float[] actions)
        {
            //Debug.Log("Action");
            nextActions = actionScale * ActionsToVector(actions, activeDofLocalIndices, dofAddresses.Length);

        }

        public float[] GetActionsFromState()
        {
            /*
            if (kinematicRef && !trackState)
            {
                float[] refs = activeReferenceStates.SelectMany(rs => rs.PositionErrors).Select(p => -(float)(p / actionScale)).ToArray();
                //Debug.Log("we have: " + refs.Length + "  " + ActionSpaceSize);
                return refs;
            }
            else
            {
                return Enumerable.Repeat(0f, ActionSpaceSize).ToArray();
            }
            */
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

        public unsafe void MjInitialize()
        {
            MjScene.Instance.ctrlCallback += UpdateTorque;
            var actuatedJoints = ActuatedJoints;
            var controlledJoints = ControlledJoints;

            Func<MjBaseJoint, bool> IsActive = (MjBaseJoint j) => !IsExcludeDefined || controlledJoints.Contains(j);
            Func<IEnumerable<MjBaseJoint>, IEnumerable<bool>> GetDofActivity = (IEnumerable<MjBaseJoint> js) => js.Select(j => Enumerable.Repeat(IsActive(j), j.DofCount())).SelectMany(x => x);
            Func<IEnumerable<bool>, IEnumerable<int>> GetIndicesOfTrue = (IEnumerable<bool> bs) => bs.Select((b, i) => (b, i)).Where(enumerated => enumerated.b).Select(enumerated => enumerated.i);
            Func<IEnumerable<MjBaseJoint>, IEnumerable<int>> GetActiveDofIndices = (IEnumerable<MjBaseJoint> js) => GetIndicesOfTrue(GetDofActivity(js));

            if (kinematicRef && trackState)  // We'll assign the kinematic reference joints.
            {
                jointStates = actuatedJoints.Select(j => IMjJointState.GetJointState(j, FindReference(j))).ToList();
                activeReferenceStates = jointStates.Where(js => IsActive(js.Joint)).Select(js => js.ReferenceState).ToList();
            }
            else  // We'll let the joint states generate placeholder references (PosError will just return -Pos).
            {
                jointStates = actuatedJoints.Select(j => IMjJointState.GetJointState(j)).ToList();

                activeReferenceStates = jointStates.Where(js => IsActive(js.Joint)).Select(js => FindReference(js.Joint)).ToList();
            }

            activeDofLocalIndices = GetActiveDofIndices(actuatedJoints).ToArray();
            dofAddresses = IMjJointState.GetDofAddresses(jointStates);

            nextActions = actionScale * ActionsToVector(GetActionsFromState(), activeDofLocalIndices, dofAddresses.Length);
            
            posGainMatrix = Matrix<double>.Build.Diagonal(dofAddresses.Length, dofAddresses.Length, posGain);
            velGainMatrix = Matrix<double>.Build.Diagonal(dofAddresses.Length, dofAddresses.Length, velGain);

            inertiaSubMatrixMap = MjState.GetInertiaSubMatrixIndexMap(dofAddresses.ToList(), new MjStepArgs(MjScene.Instance.Model, MjScene.Instance.Data));
            dt = Time.fixedDeltaTime;
        }

        private void OnDisable()
        {
            if (MjScene.InstanceExists) MjScene.Instance.ctrlCallback -= UpdateTorque;
        }

        private IMjJointState FindReference(MjBaseJoint joint)
        {
            if(!kinematicRef) return null;

            if (kinematicRef.GetComponentInChildren<MjBaseJoint>())
            {
                return IMjJointState.GetJointState(kinematicRef.GetComponentsInChildren<MjBaseJoint>().First(rj => rj.name.Contains(joint.name)));
            }
            else if (kinematicRef.GetComponentInChildren<MjFiniteDifferenceJoint>())
            {
                return IMjJointState.GetJointState(kinematicRef.GetComponentsInChildren<MjFiniteDifferenceJoint>().First(rj => rj.name.Contains(joint.name)).transform);

            }
            else if (kinematicRef.GetComponentInChildren<MjMocapJointStateComponent>())
            {
                return IMjJointState.GetJointState(kinematicRef.GetComponentsInChildren<MjMocapJointStateComponent>().First(rj => rj.name.Contains(joint.name)).transform);
            }
           
            return null;
        }

        private unsafe void Start()
        {
            MjState.ExecuteAfterMjStart(MjInitialize);
        }

        public void SetStiffnessMatrix(Matrix<double> stiffness)
        {
            posGainMatrix = stiffness;
        }

        public void SetDampingMatrix(Matrix<double> damping)
        {
            velGainMatrix = damping;
        }

        private unsafe void Awake()
        {
            SetPreviousActions();
        }

        public void SetPreviousActions()
        {

            if (agent)
            {

                if (smoothingObject)
                {
                    prevActionSource = smoothingObject.GetComponent<IRememberPreviousActions>();
                    agent.OnBegin += (object sender, EventArgs e) => prevActionSource.SetPreviousActions(GetActionsFromState());
                }
            }

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

        public IActuator actuator;
        public override IActuator[] CreateActuators()
        {
            if (actuator == null)
            {
                actuator = new SPDActuator(this);
            }
            return new[] { actuator };
        }

        class SPDActuator : IActuator
        {
            SPDActuatorComponent component;

            public SPDActuator(SPDActuatorComponent component)
            {
                this.component = component;
                actionSpec = new ActionSpec(component.ActionSpaceSize);
            }

            ActionSpec actionSpec;

            public string Name => component.name + "_SPD_Actuator";

            ActionSpec IActuator.ActionSpec => actionSpec;

            public void Heuristic(in ActionBuffers actionBuffersOut)
            {
                if (!component.useHeuristic) return;
                var actions = actionBuffersOut.ContinuousActions;
                var actionsFromState = component.GetActionsFromState();
                for (var actionIndex = 0; actionIndex < actions.Length; actionIndex++)
                {
                    actions[actionIndex] = actionsFromState[actionIndex];
                }

                
            }

            public void OnActionReceived(ActionBuffers actionBuffers)
            {
                var actions = actionBuffers.ContinuousActions;
                component.ApplyActions(actions.Array[actionBuffers.ContinuousActions.Offset..(actionBuffers.ContinuousActions.Offset+ actionBuffers.ContinuousActions.Length)]);
            }

            public void ResetData()
            {
                
            }

            public void WriteDiscreteActionMask(IDiscreteActionMask actionMask)
            {

            }
        }
    }
}
