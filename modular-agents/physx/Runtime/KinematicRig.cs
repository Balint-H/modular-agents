using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

using UnityEngine;
using ModularAgents.Kinematic;
using Unity.Mathematics;
using ModularAgents.Kinematic.PhysX;

namespace ModularAgents.Kinematic
{ 
/// <summary>
/// The kinematic rig maps the source avatar's movement to the standard MarathonController hierarchy, so properties of the kinematic controller's segments can be queried.
/// A new class inheriting from KinematicRig should be implemented for new animation -> ragdoll mapping.
/// </summary>
public class KinematicRig : MonoBehaviour, IKinematicReference
{
    [SerializeField]
    private Transform kinematicRagdollRoot;

    [SerializeField]
    private Transform trackedTransformRoot;

    [SerializeField]
    private Vector3 offset;

    [SerializeField]
    private bool initializeAlone = false;

    [SerializeField]
    private string Prefix = "K_";


    // Private, since other scripts should reference rigidbodies from the hierarchy, and not depend on KinematicRig implementation if possible


    private IReadOnlyList<Rigidbody> riggedRigidbodies;
    private IReadOnlyList<Transform> trackedTransforms;


    //to debug, you might want:
    //[SerializeField]
    //private List<Rigidbody> riggedRigidbodies;

    //[SerializeField]
    //private List<Transform> trackedTransforms;



    public IReadOnlyList<Transform> RagdollTransforms => riggedRigidbodies.Select(rb => rb.transform).ToList();

    public IEnumerable<Rigidbody> Rigidbodies => riggedRigidbodies;


    public IReadOnlyList<Vector3> RagdollLinVelocities => riggedRigidbodies.Select(rb => rb.velocity).ToList();

    public IReadOnlyList<Vector3> RagdollAngularVelocities => riggedRigidbodies.Select(rb => rb.angularVelocity).ToList();

  
    
    public IReadOnlyList<IKinematic> Kinematics => riggedRigidbodies.Select(rb => (IKinematic) new RigidbodyAdapter(rb)).ToList();


    public Transform KinematicRagdollRoot { get => kinematicRagdollRoot; }
    public Transform TrackedTransformRoot { get => trackedTransformRoot; }


    public (float3, float3) RootVelocities() { 
       Rigidbody rootRB = riggedRigidbodies.First((rb) => rb.name.Equals(kinematicRagdollRoot.name));
        return (rootRB.angularVelocity, rootRB.velocity);


    
    } 

    void Awake() 
    
    {

       if( initializeAlone)
            OnAgentInitialize();


    }


    // Update is called once per frame



    void FixedUpdate()
    {
        TrackKinematics();
    }

    public void OnAnimatorIK(int k)
    {
        TrackKinematics();
    }
    public void OnAgentInitialize()
    {
        riggedRigidbodies = kinematicRagdollRoot.GetComponentsInChildren<Rigidbody>();
        trackedTransforms = trackedTransformRoot.GetComponentsInChildren<Transform>();

        //riggedRigidbodies = kinematicRagdollRoot.GetComponentsInChildren<Rigidbody>().ToList();
        //trackedTransforms = trackedTransformRoot.GetComponentsInChildren<Transform>().ToList();


      //  (riggedRigidbodies, trackedTransforms) = MarathonControllerMapping(riggedRigidbodies, trackedTransforms); //Only transfroms that have corresponding RB are tracked
        (riggedRigidbodies, trackedTransforms) = PrefixControllerMapping(riggedRigidbodies, trackedTransforms);
    }

    public void TrackKinematics()
    {
        
        foreach((var rb, var tr) in riggedRigidbodies.Zip(trackedTransforms, Tuple.Create))
        {
 

            rb.MoveRotation(tr.rotation);
            rb.MovePosition(tr.position + offset);

//          rb.rotation = tr.rotation;
  //        rb.position = tr.position + offset;
        }
    }

    public void TeleportRoot(Vector3 position, Quaternion rotation)
    {
        Vector3 positionalOffset = position - riggedRigidbodies[0].position;
        Quaternion rotationalOffset = Quaternion.Inverse(riggedRigidbodies[0].rotation) * rotation;

        IEnumerable<KinematicState> kinematicStates = riggedRigidbodies.Select(rb => new KinematicState(rb.angularVelocity, rb.velocity, rb.position, rb.rotation));
        foreach((Rigidbody rb, KinematicState kinState) in riggedRigidbodies.Zip(kinematicStates, Tuple.Create))
        {
            rb.position = kinState.position + positionalOffset;
            rb.rotation = kinState.rotation * rotationalOffset;

            //The instructions below are not supported.
            //rb.velocity = rotationalOffset * kinState.linearVelocity;
           // rb.angularVelocity = rotationalOffset * kinState.angularVelocity;
        }
    }

    public void TeleportRoot(Vector3 position)
    {
        Vector3 positionalOffset = position - riggedRigidbodies[0].position;

        IEnumerable<KinematicState> kinematicStates = riggedRigidbodies.Select(rb => new KinematicState(rb.angularVelocity, rb.velocity, rb.position, rb.rotation));
        foreach ((Rigidbody rb, KinematicState kinState) in riggedRigidbodies.Zip(kinematicStates, Tuple.Create))
        {
            rb.position = kinState.position + positionalOffset;
            rb.velocity = kinState.linearVelocity;
            rb.angularVelocity = kinState.angularVelocity;
        }
    }

    private (IReadOnlyList<Rigidbody>, IReadOnlyList<Transform>) MarathonControllerMapping(IReadOnlyList<Rigidbody> rigidbodies, IReadOnlyList<Transform> transforms)
    {
        List<string> rigidbodyNames = riggedRigidbodies.Select(rb => Utils.SegmentName(rb.name)).ToList();
        transforms = transforms.Where(t => rigidbodyNames.Contains(Utils.SegmentName(t.name))).ToList();

        return (rigidbodies, transforms);
    }



     private (IReadOnlyList<Rigidbody>, IReadOnlyList<Transform>) PrefixControllerMapping(IReadOnlyList<Rigidbody> rigidbodies, IReadOnlyList<Transform> transforms)
    //private (List<Rigidbody>, List<Transform>) PrefixControllerMapping(List<Rigidbody> rigidbodies, List<Transform> transforms)
    {
    
        
        List<Transform> orderedTransforms = new List<Transform>();

        foreach (Rigidbody rb in rigidbodies)
            orderedTransforms.Add(transforms.First(t => t.name.Contains( (rb.name).Replace(Prefix, "") ) )) ;
        

        return (rigidbodies, orderedTransforms);

     
    }


    private struct KinematicState
    {
        public readonly Vector3 angularVelocity;
        public readonly Vector3 linearVelocity;
        public readonly Vector3 position;
        public readonly Quaternion rotation;

        public KinematicState(Vector3 angularVelocity, Vector3 linearVelocity, Vector3 position, Quaternion rotation)
        {
            this.angularVelocity = angularVelocity;
            this.linearVelocity = linearVelocity;
            this.position = position;
            this.rotation = rotation;
        }
    }
}
}
