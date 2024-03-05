using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Mujoco;
using static Mujoco.Extensions.MjState;
using MathNet.Numerics.LinearAlgebra;
using System;

namespace ModularAgents.MotorControl
{
    public class HingeState : IJointState, IMjJointState
    {
        readonly MjHingeJoint hinge;
        readonly MjScene mjScene;
        readonly IMjJointState reference;
        readonly int[] dofAddresses;
        readonly int[] posAddresses;

        public HingeState(MjHingeJoint hinge, IMjJointState reference)
        {
            this.hinge = hinge;
            mjScene = MjScene.Instance;
            this.reference = reference;
            dofAddresses = new[] { hinge.DofAddress };
            posAddresses = new[] { hinge.QposAddress };
        }

        public HingeState(MjHingeJoint hinge, MjHingeJoint reference) : this(hinge, new HingeState(reference)) { }

        public HingeState(MjHingeJoint hinge) : this(hinge, new ZeroHingeState()) { }

        protected HingeState()
        {
            mjScene = MjScene.Instance;
        }

        public virtual unsafe double[] Accelerations => new[] { mjScene.Data->qacc[hinge.DofAddress] };

        public virtual unsafe double[] Velocities => new[] { mjScene.Data->qvel[hinge.DofAddress] };

        public virtual unsafe double[] Positions => new[] { mjScene.Data->qpos[hinge.QposAddress] };

        public virtual string Name => hinge.name;

        public virtual GameObject gameObject => hinge.gameObject;

        public virtual int[] DofAddresses => dofAddresses;

        public virtual int[] PosAddresses => posAddresses;

        public double[] PositionErrors => new[] {reference.Positions[0] - Positions[0]};

        public double[] VelocityErrors => new[] {reference.Velocities[0] - Velocities[0]};

        public unsafe double[] TargetPositions => reference.Positions;

        public IMjJointState ReferenceState => reference;

        public MjBaseJoint Joint => hinge;

        public static HingeState Zero { get => new ZeroHingeState(); }


        /// <summary>
        /// Used when no reference is available. In these cases, for example, calling PosError in the original joint state will return its -position.
        /// This allows shared code for the tracking and non-tracking cases.
        /// </summary>
        private class ZeroHingeState : HingeState
        {
            private double[] zeroArr;
            private int[] invalidArr;

            public ZeroHingeState()
            {
                zeroArr = new[] { 0.0 };
                invalidArr = new[] { -1 };
            }

            public override double[] Accelerations => zeroArr;

            public override double[] Velocities => zeroArr;

            public override double[] Positions => zeroArr;

            public override string Name => "ZeroHingeState";

            public override int[] DofAddresses => invalidArr;

            public override int[] PosAddresses => invalidArr;


        }
        public double[] GetStablePositionErrors(double dt)
        {
            var vels = Velocities;
            var posError = PositionErrors;
            double result = posError[0] - vels[0] * dt;
            if (double.IsNaN(result))
            {
                Debug.LogWarning("Hinge object:" + Name + "has values: pos " + posError[0] + "and vel: " + vels[0]);
            }
            return new[] { result};
        }

        public double[] GetStableVelocityErrors(double dt)
        {
            var accs = Accelerations;
            var velError = VelocityErrors;
            return new[] { velError[0] - accs[0] * dt };
        }
    }

    public class BallState : IJointState, IMjJointState
    {
        protected readonly MjBallJoint ball;
        readonly IMjJointState reference;
        readonly MjScene mjScene;
        readonly protected int[] dofAddresses;
        readonly protected int[] posAddresses;

        public BallState(MjBallJoint ball, MjBallJoint refJoint) : this(ball, new BallState(refJoint)) { }
        public BallState(MjBallJoint ball, IMjJointState refJointState)
        {
            this.ball = ball;
            mjScene = MjScene.Instance;
            dofAddresses = new[] { ball.DofAddress, ball.DofAddress + 1, ball.DofAddress + 2 };
            posAddresses = new[] { ball.QposAddress, ball.QposAddress + 1, ball.QposAddress + 2, ball.QposAddress + 3 };
            reference = refJointState;
        }

        public BallState(MjBallJoint ball): this(ball, new ZeroBallState()) { }

        protected BallState(int[] dofAddresses, int[] posAddresses)
        {
            mjScene = MjScene.Instance;
            this.dofAddresses = dofAddresses;
            this.posAddresses = posAddresses;
        }

        // The joint state in generalized joint space in Unity format. May not be the same as the local default Unity quaternion.
        public unsafe virtual Quaternion GeneralizedUnityQuaternion { get => MjEngineTool.UnityQuaternion(mjScene.Data->qpos + ball.QposAddress); }

        public unsafe virtual double[] Accelerations => Utils.ToArray(mjScene.Data->qacc + ball.DofAddress, 3);

        public unsafe virtual double[] Velocities => Utils.ToArray(mjScene.Data->qvel + ball.DofAddress, 3);

        public unsafe virtual double[] Positions => Utils.ToArray(mjScene.Data->qpos + ball.QposAddress, 4);

        public unsafe virtual double[] TargetPositions => reference.Positions;

        public virtual string Name => ball.name;

        public GameObject gameObject => ball.gameObject;

        public virtual int[] DofAddresses => dofAddresses;

        public virtual int[] PosAddresses => posAddresses;

        public double[] PositionErrors => Utils.QuaternionError(Positions, reference.Positions);

        public MjBaseJoint Joint => ball;

        public double[] VelocityErrors 
        { 
            get
            {
                double[] curV = Velocities;
                double[] desV = reference.Velocities;
                return new[] { desV[0] - curV[0], desV[1] - curV[1], desV[2] - curV[2] };
            } 
        }

        public IMjJointState ReferenceState => reference;

        public static BallState Zero { get => new ZeroBallState(); }

        /// <summary>
        /// Used when no reference is available. In these cases, for example, calling PosError in the original joint state will return its -position.
        /// This allows shared code for the tracking and non-tracking cases.
        /// </summary>
        private class ZeroBallState: BallState
        {

            public override Quaternion GeneralizedUnityQuaternion => Quaternion.identity;
            private double[] zeroDof;
            private double[] zeroPos;
            public override double[] Velocities => zeroDof;

            public override double[] Accelerations => zeroDof;

            public override double[] Positions => zeroPos;

            public override string Name => "ZeroBallState";

            public ZeroBallState(): base(dofAddresses: new[] { -1, -1, -1 }, posAddresses: new[] { -1, -1, -1, -1 })
            {
                zeroDof = new[] { 0.0, 0, 0 };
                zeroPos = new[] { 1.0, 0, 0, 0 };
                
            }

        }

        public double[] GetStablePositionErrors(double dt)
        {
            var vels = Velocities;
            var posError = PositionErrors;

            double[] result = new double[3] { posError[0] - vels[0] * dt, posError[1] - vels[1] * dt, posError[2] - vels[2] * dt };

            int i = 0;
            foreach (double d in result)
            {
                if (double.IsNaN(d))
                {
                    Debug.LogWarning("Ball object:" + Name + "has values: pos " + posError[i] + "and vel: " + vels[i] + " on axis " + i);
                }
 
                i++;
            }


            return  result;
        }

        public double[] GetStableVelocityErrors(double dt)
        {
            var accs = Accelerations;
            var velError = VelocityErrors;
            return new[] { velError[0] - accs[0] * dt, velError[1] - accs[1] * dt, velError[2] * accs[2] * dt };
        }
    }


    public class FreeJointState : IJointState, IMjJointState
    {
        MjFreeJoint joint;
        FreeJointState reference;
        MjScene mjScene;
        int[] dofAddresses;
        int[] posAddresses;

        public FreeJointState(MjFreeJoint joint, MjFreeJoint refJoint)
        {
            this.joint = joint;
            mjScene = MjScene.Instance;
            dofAddresses = Enumerable.Range(0, 6).Select(i => joint.DofAddress+i).ToArray();
            posAddresses = Enumerable.Range(0, 7).Select(i => joint.QposAddress + i).ToArray();
            reference = new FreeJointState(refJoint);
            throw new System.NotImplementedException();
        }

        public FreeJointState(MjFreeJoint joint)
        {
            this.joint = joint;
            mjScene = MjScene.Instance;
            dofAddresses = Enumerable.Range(0, 6).Select(i => joint.DofAddress + i).ToArray();
            posAddresses = Enumerable.Range(0, 7).Select(i => joint.QposAddress + i).ToArray();
            throw new System.NotImplementedException();

        }


        public double[] Accelerations => throw new System.NotImplementedException();

        public double[] Velocities => throw new System.NotImplementedException();

        public double[] Positions => throw new System.NotImplementedException();

        public double[] PositionErrors => throw new System.NotImplementedException();

        public double[] VelocityErrors => throw new System.NotImplementedException();

        public string Name => throw new System.NotImplementedException();

        public GameObject gameObject => throw new System.NotImplementedException();

        public int[] DofAddresses => throw new System.NotImplementedException();

        public int[] PosAddresses => throw new System.NotImplementedException();

        public double[] TargetPositions => throw new System.NotImplementedException();

        public MjBaseJoint Joint => throw new System.NotImplementedException();

        public IMjJointState ReferenceState => throw new System.NotImplementedException();

        public double[] GetStablePositionErrors(double dt)
        {
            throw new System.NotImplementedException();
        }

        public double[] GetStableVelocityErrors(double dt)
        {
            throw new System.NotImplementedException();
        }
    }
    public interface IMjJointState: IJointState
    {
        public unsafe int[] DofAddresses { get; }
        public unsafe int[] PosAddresses { get; }

        public double[] TargetPositions { get; }

        public MjBaseJoint Joint { get; }

        public IMjJointState ReferenceState { get; }
        public static IMjJointState GetJointState(MjBaseJoint joint)
        {
            switch (joint)
            {
                case MjHingeJoint hinge:
                    return new HingeState(hinge);

                case MjBallJoint ball:
                    return new BallState(ball);

                default:
                    throw new System.NotImplementedException("Joint type not implemented yet");

            }
        }
        public static IMjJointState GetJointState(MjBaseJoint joint, MjBaseJoint reference)
        {
            switch (joint)
            {
                case MjHingeJoint hinge:
                    return new HingeState(hinge, (MjHingeJoint) reference);

                case MjBallJoint ball:
                    return new BallState(ball, (MjBallJoint) reference);

                default:
                    throw new System.NotImplementedException("Joint type not implemented yet");

            }
        }

        public static IMjJointState GetJointState(MjBaseJoint joint, IMjJointState reference)
        {
            switch (joint)
            {
                case MjHingeJoint hinge:
                    return new HingeState(hinge, reference);

                case MjBallJoint ball:
                    return new BallState(ball, reference);

                default:
                    throw new System.NotImplementedException("Joint type not implemented yet");

            }
        }



        public static IMjJointState GetZeroJointStateLike(MjBaseJoint joint)
        {
            switch (joint)
            {
                case MjHingeJoint hinge:
                    return HingeState.Zero;

                case MjBallJoint ball:
                    return BallState.Zero;

                default:
                    throw new System.NotImplementedException("Joint type not implemented yet");

            }
        }

        public double[] GetStablePositionErrors(double dt);
        public double[] GetStableVelocityErrors(double dt);

        public static IMjJointState GetJointState(GameObject go)
        {
            return GetJointState(go.transform);
        }

        public static IMjJointState GetJointState(Transform transform)
        {
            if (transform.GetComponent<MjActuator>())
            {
                return GetJointState(transform.GetComponent<MjActuator>());
            }
            else if (transform.GetComponent<MjBaseJoint>())
            {
                return GetJointState(transform.GetComponent<MjBaseJoint>());
            }
            else if (transform.GetComponent<MjFiniteDifferenceJoint>())
            {
                return transform.GetComponent<MjFiniteDifferenceJoint>().GetJointState();
            }
            else if(transform.GetComponent<MjMocapJointStateComponent>())
            {
                return transform.GetComponent<MjMocapJointStateComponent>().GetIJointState();
            }
            throw new NotImplementedException($"No component on transform {transform.name} can be interpreted as a JointState.");
        }

        public static IMjJointState GetJointState(MjActuator act)
        {
            return GetJointState(act.Joint);
        }

        public static Vector<double> GetPosErrorVector(IEnumerable<IMjJointState> jointStates)
        {
            var errArray = jointStates.SelectMany(js => js.PositionErrors);
            var err = Vector<double>.Build.DenseOfEnumerable(errArray);
            return err;
        }

        public static Vector<double> GetStablePosErrorVector(IEnumerable<IMjJointState> jointStates, double dt)
        {
            var errArray = jointStates.SelectMany(js => js.GetStablePositionErrors(dt));
            var err = Vector<double>.Build.DenseOfEnumerable(errArray);
            return err;
        }

        public static Vector<double> GetVelErrorVector(IEnumerable<IMjJointState> jointStates)
        {
            var errArray = jointStates.SelectMany(js => js.VelocityErrors);
            var err = Vector<double>.Build.DenseOfEnumerable(errArray);
            return err;
        }


        public static int[] GetDofAddresses(IEnumerable<IMjJointState> jointStates)
        {
            return jointStates.SelectMany(js => js.DofAddresses).ToArray();
        }

        public static int[] GetDofAddresses(IEnumerable<MjBaseJoint> joints)
        {
            return GetDofAddresses(joints.Select(j => IMjJointState.GetJointState(j)));
        }


    }

}