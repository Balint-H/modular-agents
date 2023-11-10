using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Mujoco;
using static Mujoco.Extensions.Scaling;
using ModularAgents.Kinematic.Mujoco;
using System.Linq;

[CustomEditor(typeof(MjFiniteDifferenceManager))]
public class MjFiniteDifferenceManagerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        base.OnInspectorGUI();


        GUILayout.BeginHorizontal();

        string prefix = EditorGUILayout.TextField(new GUIContent("Prefix", "Optional prefix that will be included in the name matching of MjBodies and prefixed animated transform names."), "");


        if (GUILayout.Button("Generate Finite Difference Components"))
        {
            MjFiniteDifferenceManager t = target as MjFiniteDifferenceManager;
            RecursiveComponentCreation(t.Root.GetComponentInParent<MjBody>(), t.Animator.transform, prefix);
        }

        GUILayout.EndHorizontal();
        serializedObject.ApplyModifiedProperties();

    }

    private void RecursiveComponentCreation(MjBody mjBody, Transform parentTransform, string prefix)
    {
        var childTransforms = parentTransform.GetComponentsInChildren<Transform>().Where(t => prefix+t.name == mjBody.name);
        if (childTransforms.Count() > 1)
        {
            Debug.LogWarning($"More than 1 match found for body {mjBody.name}: {string.Join(", ", childTransforms.Select(t => t.name))} Kinematic rig creation would likely fail.");
            return;
        }
        if (childTransforms.Count() < 1)
        {
            Debug.LogWarning($"No match found for body {mjBody.name}. The corresponding animated transform is expected to share the name of the MjBody.");
            return;
        }
        var finiteDifferenceBody = childTransforms.First().gameObject.AddComponent<MjFiniteDifferenceBody>();
        //finiteDifferenceBody.PairedBody = mjBody;

        foreach (var joint in mjBody.GetBodyChildComponents<MjBaseJoint>())
        {
            var finiteDifferenceJoint = new GameObject(prefix + joint.name).AddComponent<MjFiniteDifferenceJoint>();
            finiteDifferenceJoint.transform.SetLocalPositionAndRotation(joint.transform.localPosition, joint.transform.localRotation);
            finiteDifferenceJoint.transform.parent = finiteDifferenceBody.transform;
            finiteDifferenceJoint.PairedJoint = joint;
        }
        foreach (var childBody in mjBody.GetBodyChildComponents<MjBody>())
        {
            RecursiveComponentCreation(childBody, finiteDifferenceBody.transform, prefix);
        }
    }
}
