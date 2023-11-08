using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;

namespace ModularAgents.Kinematic
{

    public static class KinematicExtensions
    {
        
        public static DReCon.ReferenceFrame GetReferenceFrame(this IKinematic kinematic)
        {
            return new DReCon.ReferenceFrame(kinematic.Forward, kinematic.Position);
        }

        public static DReCon.ReferenceFrame GetCentreOfMassReferenceFrame(this IKinematic kinematic)
        {
            return new DReCon.ReferenceFrame(kinematic.Forward, kinematic.CenterOfMass);
        }

        public static DReCon.ReferenceFrame GetCentreOfMassReferenceFrame(this BodyChain kinematicChain)
        {
            return new DReCon.ReferenceFrame(kinematicChain.RootForward, kinematicChain.CenterOfMass);
        }


    }


    /// <summary>
    /// Provides access to COM properties of ArticulationBodies or Rigidbodies arranged in a kinematic chain hierarchy
    /// </summary>
    public abstract class BodyChain: IEnumerable<IKinematic>
    {
        protected IReadOnlyList<IKinematic> chain;

        protected float mass; //chain.Select(k => k.Mass).Sum();
        public float Mass {get => mass;}
        public Vector3 CenterOfMass {get => chain.Select(k => k.Mass * k.CenterOfMass).Sum() / Mass;}
        public Vector3 CenterOfMassVelocity {get => chain.Select(k => k.Mass * k.Velocity).Sum() / Mass;}
        public IEnumerable<Vector3> CentersOfMass {get => chain.Select(k => k.CenterOfMass);}
        public IEnumerable<Vector3> Velocities {get => chain.Select(k => k.Velocity);}
        public IEnumerable<Matrix4x4> TransformMatrices { get => chain.Select(k => k.TransformMatrix); }
        public Vector3 RootForward { get => chain[0].Forward; }

        public IEnumerable<string> Names { get => chain.Select(k => k.Name); }

        public IEnumerable<IKinematic> Links { get => chain; }

        public IKinematic Root { get => chain[0]; }
        public BodyChain() { }

        public BodyChain(Transform chainRoot)
        {
            chain = GetKinematicChain(chainRoot);

            mass = chain.Select(k => k.Mass).Sum();
        }


        public BodyChain(IEnumerable<Transform> bodies) {
          throw new  NotImplementedException("the inheritance class needs to implement the construction of the chain");
        
        }


        protected abstract  IReadOnlyList<IKinematic> GetKinematicChain(Transform root);

        public IEnumerator<IKinematic> GetEnumerator()
        {
            return chain.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return chain.GetEnumerator();
        }

        public IKinematic this[int index]
        {
            get => chain[index];
        }

        public int Count => chain.Count;


    }


    public interface IKinematic
    {
        public Vector3 Velocity { get; }

        public Vector3 AngularVelocity { get; }
        public float Mass { get; }
        public Vector3 CenterOfMass { get; }

        public Matrix4x4 TransformMatrix { get; }


        int index { get; } //position in the hierarchy
        public Quaternion Rotation { get; }
        public Quaternion LocalRotation { get; }

        public Vector3 Position { get; }

        /// <summary>
        /// Cartesian linear velocity, defined in the parent frame.
        /// </summary>
        public Vector3 LocalVelocity { get; }

        /// <summary>
        /// Cartesian angular velocity, defined in the parent frame.
        /// </summary>
        public Vector3 LocalAngularVelocity { get; }

        public Vector3 GetPointVelocity(Vector3 worldPoint);

        public Vector3 GetRelativePointVelocity(Vector3 localPoint);
        public string Name { get; }

        public GameObject gameObject { get; }

        public Vector3 Forward { get; }

        //public Matrix4x4 LocalTransformMatrix { get; }
    }

    /// <summary>
    /// The identity adapter provides constant fields that when a second IKinematic's state is compared with it, the difference will be the second state.
    /// This is useful for example, when unifying absolute and relative controllers, as an absolute controller can be relative to this IKinematic.
    /// The name refers to the fields of this classes (mostly) being identity elements of their corresponding vector spaces.
    /// </summary>
    public class IdentityAdapter : IKinematic
    {
        public Vector3 Velocity => Vector3.zero;

        public Vector3 AngularVelocity => Vector3.zero;

        public float Mass => 0;

        public Vector3 Position => Vector3.zero;

        public Vector3 CenterOfMass => Vector3.zero;

        public Matrix4x4 TransformMatrix => Matrix4x4.identity;

        public string Name => "identity";
        public int index { get => -1; }
        public GameObject gameObject => throw new System.NotImplementedException();

        public Vector3 Forward => Vector3.forward;

        public Matrix4x4 LocalTransformMatrix => Matrix4x4.identity;

        public Vector3 GetPointVelocity(Vector3 worldPoint)
        {
            return worldPoint;
        }

        public Vector3 GetRelativePointVelocity(Vector3 localPoint)
        {
            return localPoint;
        }

        public Quaternion Rotation => Quaternion.identity;
        public Quaternion LocalRotation => Quaternion.identity;

        public Vector3 LocalVelocity => Vector3.zero;

        public Vector3 LocalAngularVelocity => Vector3.zero;
    }

}

