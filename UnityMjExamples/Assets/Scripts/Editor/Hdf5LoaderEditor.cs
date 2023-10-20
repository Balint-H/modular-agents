using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using ModularAgents.Kinematic;
using Mujoco;
using Mujoco.Extensions;

[CustomEditor(typeof(Hdf5Loader))]
public class Hdf5LoaderEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        base.OnInspectorGUI();

        if (GUILayout.Button("Generate Data View Components"))
        {
            Hdf5Loader t = target as Hdf5Loader;
            RecursiveDataViewCreation(t.Root.GetComponentInParent<MjBody>(), t.transform, t);
        }
        serializedObject.ApplyModifiedProperties();

    }

    private void RecursiveDataViewCreation(MjBody mjBody, Transform parentTransform, Hdf5Loader dataLoader)
    {
        var bodyView = new GameObject("Mocap_"+mjBody.name).AddComponent<MjMocapBodyKinematicsComponent>();
        bodyView.transform.parent = parentTransform;
        bodyView.transform.SetPositionAndRotation(mjBody.transform.position, mjBody.transform.rotation);
        bodyView.PairedBody = mjBody;
        bodyView.DataLoader = dataLoader;
        foreach (var joint in mjBody.GetBodyChildComponents<MjBaseJoint>())
        {
            var jointView = new GameObject("Mocap_"+joint.name).AddComponent<MjMocapJointStateComponent>();
            jointView.transform.SetPositionAndRotation(joint.transform.position, joint.transform.rotation);
            jointView.transform.parent = bodyView.transform;
            jointView.PairedJoint = joint;
            jointView.DataLoader = dataLoader;
        }
        foreach(var childBody in mjBody.GetBodyChildComponents<MjBody>())
        {
            RecursiveDataViewCreation(childBody, bodyView.transform, dataLoader);
        }
    }
}
