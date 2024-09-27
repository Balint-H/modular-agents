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
public class MjKinematicRig : MonoBehaviour, IKinematicReference
{
    [SerializeField]
    private Transform weldRoot;

    [SerializeField]
    private Transform trackedTransformRoot;

    [SerializeField]
    private Vector3 offset;

    [SerializeField]
    private string prefix;

    [SerializeField]
    [FormerlySerializedAs("ragdollRoot")]
    private Transform kinematicRagdollRoot;

    [SerializeField]
    private int targetId;

    [SerializeField]
    private bool intializeAlone;

    // Private, since other scripts should reference rigidbodies from the hierarchy, and not depend on KinematicRig implementation if possible
    private IReadOnlyList<Transform> riggedTransforms;
    private IReadOnlyList<Transform> trackedTransforms;

    public IReadOnlyList<Transform> RagdollTransforms => bodies.Select(bd => bd.transform).ToList();

    private IReadOnlyList<MjMocapBody> mjMocapBodies;

    private IReadOnlyList<MjBody> bodies;

    public IReadOnlyList<MjBody> Bodies { get => bodies; }

    public IReadOnlyList<Vector3> RagdollLinVelocities => throw new NotImplementedException();

    public IReadOnlyList<Vector3> RagdollAngularVelocities => throw new NotImplementedException();

    public IReadOnlyList<IKinematic> Kinematics => kinematicRagdollRoot.GetComponentsInChildren<MjBody>().Select(mjb => (IKinematic) new MjBodyAdapter(mjb)).ToList();

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
        Func<string, string> MocapName = animatedName => $"{prefix}{Utils.SegmentName(animatedName)}";

        void AlignOrientationsPositions(List<Transform> Trackers, List<Transform> Tracked)
        {
            foreach (Transform t in Tracked)
            {
                string targetname = MocapName(t.name);
                Transform tracker = Trackers.FirstOrDefault(t => t.name.Equals(targetname));
                tracker.rotation = t.rotation;
                tracker.position = t.position;
                //Debug.Log("aligned tracker " + tracker.name + "  with tracked transform: " + t.name);
            }
        }
        
        riggedTransforms = weldRoot.GetComponentsInChildren<MjWeld>().Select(krt => krt.Body1.transform).Where(t => t.name.Contains("Mocap") && t.gameObject.activeSelf).ToList().AsReadOnly();
        trackedTransforms = riggedTransforms.Select(rt => trackedTransformRoot.GetComponentsInChildren<Transform>().First(tt => MocapName(tt.name).Equals(rt.name))).ToList().AsReadOnly();
        AlignOrientationsPositions(riggedTransforms.ToList(), trackedTransforms.ToList());
        mjMocapBodies = riggedTransforms.Select(t => t.GetComponent<MjMocapBody>()).ToList();
        bodies = kinematicRagdollRoot.GetComponentsInChildren<MjBody>().ToList();
    }

    public virtual unsafe void TrackKinematics()
    {
        foreach ((var mjb, var tr) in riggedTransforms.Zip(trackedTransforms, Tuple.Create))
        {
            mjb.position = tr.position;
            mjb.rotation = tr.rotation;
        }

        foreach (var mcbd in mjMocapBodies)
        {
            mcbd.OnSyncState(MjScene.Instance.Data);
        }

    }



    public unsafe void TeleportRoot(Vector3 position, Quaternion rotation)
    {

        Vector3 posLag = bodies[0].GlobalPosition() - riggedTransforms[0].position;
        Quaternion rotLag = bodies[0].GlobalRotation() * Quaternion.Inverse(riggedTransforms[0].rotation);
        TrackKinematics();

        if (targetId > -1)
        {
            MjState.TeleportMjRoot(targetId, posLag + position, rotLag * rotation);
            return;
        }

        MjState.TeleportMjRoot(kinematicRagdollRoot.GetComponentInChildren<MjFreeJoint>(), posLag + position, rotLag * rotation);
    }

    public unsafe void TeleportRoot(Vector3 position)
    {

        Vector3 posLag = bodies[0].GlobalPosition() - riggedTransforms[0].position;
        Quaternion rotLag = bodies[0].GlobalRotation() * Quaternion.Inverse(riggedTransforms[0].rotation);

        TrackKinematics();

        if (targetId > -1)
        {
            Debug.Log(kinematicRagdollRoot.GetComponentInChildren<MjFreeJoint>().MujocoId);
            MjState.TeleportMjRoot(targetId, posLag + position, rotLag * riggedTransforms[0].rotation);
            return;
        }


        MjState.TeleportMjRoot(kinematicRagdollRoot.GetComponentInChildren<MjFreeJoint>(), posLag + position, rotLag * riggedTransforms[0].rotation);
    }

    public unsafe void TrackKinematicsOffline()
    {


        Func<string, string> MocapName = animatedName => $"{prefix}{Utils.SegmentName(animatedName)}";
        Func<string, string> RemovePrefix = mocapName => mocapName.Replace(prefix, "");

        var riggedTransforms = weldRoot.GetComponentsInChildren<MjWeld>().Select(krt => krt.Body1.transform).Where(t => t.name.Contains(prefix) && t.gameObject.activeSelf).ToList().AsReadOnly();

        Debug.Log(string.Join(", ", weldRoot.GetComponentsInChildren<MjWeld>().Select(krt => krt.Body1.transform).Select(t => t.name.Contains(prefix))));

        var trackedTransforms = riggedTransforms.Select(rt => trackedTransformRoot.GetComponentsInChildren<Transform>().First(tt => tt.name.Contains(RemovePrefix(rt.name)))).ToList().AsReadOnly();

        foreach ((var mjb, var tr) in riggedTransforms.Zip(trackedTransforms, Tuple.Create))
        {
            mjb.position = tr.position;
            mjb.rotation = tr.rotation;
        }

        /*        foreach (var mcbd in mjMocapBodies)
                {
                    mcbd.OnSyncState(MjScene.Instance.Data);
                }*/

    }

    public void CreateMocapBodies()
    {

        foreach (var t in trackedTransformRoot.GetComponentsInChildren<Transform>())
        {
            var go = new GameObject("Mocap_K_" + Utils.SegmentName(t.name));
            go.transform.position = t.position;
            go.transform.rotation = t.rotation;
            var mocapBd = go.AddComponent<MjMocapBody>();
            go.transform.parent = kinematicRagdollRoot.parent;

            var wGo = new GameObject("weld_" + t.name);
            wGo.transform.parent = weldRoot;
            var mjW = wGo.AddComponent<MjWeld>();
            mjW.Body1 = mocapBd;
            try
            {
                mjW.Body2 = kinematicRagdollRoot.GetComponentsInChildren<MjBody>().First(rd => go.name.Equals("Mocap_" + rd.name));
            }
            catch
            {
                DestroyImmediate(wGo);
                DestroyImmediate(go);
            }
        }


    }

}
}

