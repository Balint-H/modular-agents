using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace ModularAgents.MotorControl.PhysX
{
    public interface IState
    {
        public double[] Accelerations { get; }
        public double[] Velocities { get; }
        public double[] Positions { get; }
        public double[] PositionErrors { get; }
        public double[] VelocityErrors { get; }


        public string Name { get; }

        public GameObject gameObject { get; }

    }


    public struct StaticState : IState
    {
        readonly double position;
        readonly double velocity;
        readonly double acceleration;
        readonly double refPosition;
        readonly double refVelocity;

        public StaticState(float position, float velocity, float acceleration, float refPosition=0, float refVelocity=0)
        {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.refPosition = refPosition;
            this.refVelocity = refVelocity;
        }

        public double[] Accelerations => new[] { acceleration} ;

        public double[] Velocities => new[] { velocity };

        public double[] Positions => new[] { position };

        public double[] PositionErrors => new[] { refPosition - position };
        public double[] VelocityErrors => new[] { refVelocity - velocity };

        public string Name => "ManualState";

        public GameObject gameObject => null;

        public double[] PosDiff(IState subtrahend)
        {
            return new[] {position - subtrahend.Positions[0]};
        }

        public double[] VelDiff(IState subtrahend)
        {
            return new[] { velocity - subtrahend.Velocities[0] };
        }
    }
}