using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Mujoco;
using Mujoco.Extensions;
using System;
using ModularAgents.Kinematic.Mujoco;
using UnityEngine.Serialization;
using ModularAgents.Kinematic;

namespace ModularAgents.Kinematic
{ 
public class MjKinematicRigFD : MonoBehaviour, IKinematicReference
{
   
    [SerializeField]
    private Vector3 offset;

    [SerializeField]
    [FormerlySerializedAs("ragdollRoot")]
    private Transform kinematicRagdollRoot;

    [SerializeField]
    private int targetId;

    [SerializeField]
    private bool intializeAlone;

    // Private, since other scripts should reference rigidbodies from the hierarchy, and not depend on KinematicRig implementation if possible
    private IReadOnlyList<Transform> refTransforms;


        // public IReadOnlyList<Transform> RagdollTransforms => bodies.Select(bd => bd.transform).ToList();
        public IReadOnlyList<Transform> RagdollTransforms => refTransforms.Select(bd => bd.transform).ToList();

    //    private IReadOnlyList<MjBody> bodies;

   // public IReadOnlyList<MjBody> Bodies { get => bodies; }

    public IReadOnlyList<Vector3> RagdollLinVelocities => throw new NotImplementedException();

    public IReadOnlyList<Vector3> RagdollAngularVelocities => throw new NotImplementedException();

        public IReadOnlyList<IKinematic> Kinematics => kinematicRagdollRoot.GetComponentsInChildren<MjFiniteDifferenceBody>().Select(mjb => (IKinematic) new MjFiniteDifferenceBody.FiniteDifferenceBodyKinematics(mjb)).ToList();
        //public IReadOnlyList<IKinematic> Kinematics => throw new NotImplementedException();


        public Transform KinematicRagdollRoot { get => kinematicRagdollRoot; }
  

    public void OnAnimatorIK(int k)
    {
        TrackKinematics();
    }

    private void LateUpdate()
    {
        if (intializeAlone) TrackKinematics();
    }


    private void Awake()
    {
        if (intializeAlone) OnAgentInitialize();
    }

    public void OnAgentInitialize()
    {
            refTransforms = kinematicRagdollRoot.GetComponentsInChildren<MjFiniteDifferenceBody>().Select(krt => krt.transform).ToList().AsReadOnly();
    }



        public (IEnumerable<double[]>, IEnumerable<double[]>) GetMjKinematics()
        {
            //this function follows  MjState.GetMjKinematics(MjBody root);


           //we assume that when we initialised the MjFiniteDifference we added a MjFiniteDifferenceJoint on each:
            MjFiniteDifferenceJoint[] mjfdj = kinematicRagdollRoot.GetComponentsInChildren<MjFiniteDifferenceJoint>().ToArray();


            return (mjfdj.Select(x => x.GetJointState().Positions), mjfdj.Select(x => x.GetJointState().Velocities));

        }

        public (double[], double[]) GetRootKinematics()
        {

            var rootJoint = KinematicRagdollRoot.GetComponent<MjFiniteDifferenceJoint>();
            return (rootJoint.GetJointState().Positions, rootJoint.GetJointState().Velocities);

        }





        public virtual unsafe void TrackKinematics()
    {

        /*
        foreach ((var mjb, var tr) in riggedTransforms.Zip(trackedTransforms, Tuple.Create))
        {
            mjb.position = tr.position;
            mjb.rotation = tr.rotation;
        }*/
        /*
        foreach (var mcbd in mjMocapBodies)
        {
            mcbd.OnSyncState(MjScene.Instance.Data);
        }*/

    }





    public unsafe void TeleportRoot(Vector3 position, Quaternion rotation)
    {

       // Vector3 posLag = bodies[0].GlobalPosition() - riggedTransforms[0].position;//the difference between the kinematic root in the pupeteered ragdoll and hte mocap body root
       // Quaternion rotLag = bodies[0].GlobalRotation() * Quaternion.Inverse(riggedTransforms[0].rotation);
        TrackKinematics();

        if (targetId > -1)
        {
            MjState.TeleportMjRoot(targetId, position,  rotation);
            return;
        }

       // MjState.TeleportMjRoot(kinematicRagdollRoot.GetComponentInChildren<MjFreeJoint>(), position,  rotation);
    }

    public unsafe void TeleportRoot(Vector3 position)
    {

      //  Vector3 posLag = bodies[0].GlobalPosition() - riggedTransforms[0].position;
     //   Quaternion rotLag = bodies[0].GlobalRotation() * Quaternion.Inverse(riggedTransforms[0].rotation);

        TrackKinematics();

        if (targetId > -1)
        {
            Debug.Log(kinematicRagdollRoot.GetComponentInChildren<MjFreeJoint>().MujocoId);
                // MjState.TeleportMjRoot(targetId, posLag + position, rotLag * riggedTransforms[0].rotation);
                MjState.TeleportMjRoot(targetId, position,  refTransforms[0].rotation);
                return;
        }


        MjState.TeleportMjRoot(kinematicRagdollRoot.GetComponentInChildren<MjFreeJoint>(),  position,  refTransforms[0].rotation);
    }

  

}
}

