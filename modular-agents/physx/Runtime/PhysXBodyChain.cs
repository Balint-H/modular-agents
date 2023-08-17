using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ModularAgents.Kinematic;
using Unity.Mathematics;
using System.Linq;
using System;


namespace ModularAgents.Kinematic.PhysX
{ 
public class PhysXBodyChain : BodyChain
{

    public PhysXBodyChain(Transform chainRoot):base(chainRoot)
    {
    }


    public PhysXBodyChain(IEnumerable<Transform> bodies)
    {
        chain = bodies.Select(b => b.GetComponent<ArticulationBody>() ? (IKinematic)
                                   new ArticulationBodyAdapter(b.GetComponent<ArticulationBody>()) : new RigidbodyAdapter(b.GetComponent<Rigidbody>())).ToList().AsReadOnly();

        mass = chain.Select(k => k.Mass).Sum();
    }



    //Recursively find a read-only list of IKinematic, independent if its Rigidbodies or ArticulationBodies
    protected override IReadOnlyList<IKinematic> GetKinematicChain(Transform root)
    {
        if (root.GetComponentInChildren<ArticulationBody>())
        {
            return root.GetComponentsInChildren<ArticulationBody>().Select(ab => new ArticulationBodyAdapter(ab)).ToList().AsReadOnly();
        }
        else if (root.GetComponentInChildren<Rigidbody>())
        {
            return root.GetComponentsInChildren<Rigidbody>().Select(rb => new RigidbodyAdapter(rb)).ToList().AsReadOnly();
        }
        else
        {
            throw new NotImplementedException($"No IKinematic object found within transform {root.name} with base BodyChain.");
        }

    }




}
public static class PhysXKinematicExtensions
{

    public static PhysXBodyChain GetBodyChain(this Transform transform)
    {
        return new PhysXBodyChain(transform);
    }
  
    public static PhysXBodyChain GetBodyChain(this IEnumerable<Transform> bodies)
    {
        return new PhysXBodyChain(bodies);
    }

    public static IKinematic GetIKinematic(this Transform transform)
    {
        return transform.GetComponent<ArticulationBody>() ? (IKinematic)
                                    new ArticulationBodyAdapter(transform.GetComponent<ArticulationBody>()) : new RigidbodyAdapter(transform.GetComponent<Rigidbody>());
    }

	public static bool IsIKinematic(this Transform transform)
        {
            return transform.GetComponent<ArticulationBody>() || transform.GetComponent<Rigidbody>();
        }
}


    public class RigidbodyAdapter : IKinematic
    {
        readonly protected Rigidbody rigidbody;

        public RigidbodyAdapter(Rigidbody rigidbody)
        {
            this.rigidbody = rigidbody;
        }

        public Vector3 Velocity => rigidbody.velocity;

        public Vector3 LocalVelocity => rigidbody.transform.parent.InverseTransformDirection(Velocity);

        public Vector3 AngularVelocity => rigidbody.angularVelocity;
        public Vector3 LocalAngularVelocity => rigidbody.transform.parent.InverseTransformDirection(rigidbody.angularVelocity);

        public float Mass => rigidbody.mass;

        public Vector3 CenterOfMass => rigidbody.transform.TransformPoint(rigidbody.centerOfMass);

        public string Name => rigidbody.name;

        public Matrix4x4 TransformMatrix => rigidbody.transform.localToWorldMatrix;

        //public Matrix4x4 LocalTransformMatrix => Matrix4x4.TRS(rigidbody.transform.localPosition, rigidbody.transform.localRotation, rigidbody.transform.localScale);

        public Vector3 Forward => rigidbody.transform.forward;

        public Vector3 GetPointVelocity(Vector3 worldPoint)
        {
            return rigidbody.GetPointVelocity(worldPoint);
        }

        public Vector3 GetRelativePointVelocity(Vector3 localPoint)
        {
            return rigidbody.GetRelativePointVelocity(localPoint);
        }

        public Quaternion Rotation => rigidbody.rotation;
        public Quaternion LocalRotation => rigidbody.transform.localRotation;

        public Vector3 Position => rigidbody.transform.position;

        public GameObject gameObject { get => rigidbody.gameObject; }



        public Vector3 GetPointVelocity(float3 worldPoint)
        {
            return rigidbody.GetPointVelocity(worldPoint);
        }




        public float3 JointPosition => (float3)(Mathf.Deg2Rad * Utils.GetSwingTwist(rigidbody.transform.localRotation));
        public float3 JointVelocity
        {
            get
            {
                //notice this only makes sense because the DoF axes matrix is the identitiy, i.e. the relation between the localRotaiton and the degrees of Freedom is the identity matrix

                return LocalAngularVelocity;
            }
        }


        public float3 JointAcceleration
        {
            get
            {

                Debug.LogWarning("the acceleration of the rigidbody should not matter, why are you trying to read it? I am returing null");
                return Vector3.zero;
            }
        }


        public float3x3 JointAxes { get => float3x3.identity; }

        public int index { get => -1; }
        public bool isRoot { get => false; }
        public float3 InertiaTensor { get => rigidbody.inertiaTensor; }
        public float3 AnchorPosition
        {
            get
            {
                Debug.LogWarning("you are asking an anchor for a rigidBody, which does not have an anchor, I return zero ");
                return float3.zero;
            }
        }


    }


    /*
        public class ArticulationBodyAdapter : IKinematic
         {
             readonly private ArticulationBody articulationBody;

             public ArticulationBodyAdapter(ArticulationBody articulationBody)
             {
                 this.articulationBody = articulationBody;
             }

             public Vector3 Velocity => articulationBody.velocity;
             public Vector3 LocalVelocity => articulationBody.transform.parent.InverseTransformDirection(articulationBody.velocity);

             public Vector3 AngularVelocity => articulationBody.angularVelocity;
             public Vector3 LocalAngularVelocity => articulationBody.transform.parent.InverseTransformDirection(articulationBody.angularVelocity);

             public float Mass => articulationBody.mass;

             public Vector3 CenterOfMass => articulationBody.transform.TransformPoint(articulationBody.centerOfMass);

             public Vector3 Position => articulationBody.transform.position;

             public string Name => articulationBody.name;

             public Matrix4x4 TransformMatrix => articulationBody.transform.localToWorldMatrix;

             //public Matrix4x4 LocalTransformMatrix => Matrix4x4.TRS(articulationBody.transform.localPosition, articulationBody.transform.localRotation, articulationBody.transform.localScale);

             public Vector3 GetPointVelocity(Vector3 worldPoint)
             {
                 return articulationBody.GetPointVelocity(worldPoint);
             }

             public Vector3 GetRelativePointVelocity(Vector3 localPoint)
             {
                 return articulationBody.GetRelativePointVelocity(localPoint);
             }

             public Vector3 Forward => articulationBody.transform.forward;
             public GameObject gameObject { get => articulationBody.gameObject; }

             public Quaternion Rotation => articulationBody.inertiaTensorRotation;
             public Quaternion LocalRotation => articulationBody.parentAnchorRotation;
         }

    */
}