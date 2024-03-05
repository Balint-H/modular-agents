using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Mujoco;
using Mujoco.Extensions;
using ModularAgents.Kinematic.Mujoco;
using System.Linq;
using Unity.VisualScripting.YamlDotNet.Core.Tokens;

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
            MjBody mjBody = t.Root.GetComponentInParent<MjBody>();
            RecursiveComponentCreation(t,mjBody,  t.Animator.transform, prefix);
        }

        GUILayout.EndHorizontal();
        serializedObject.ApplyModifiedProperties();

    }

    private void RecursiveComponentCreation(MjFiniteDifferenceManager tar,MjBody mjBody, Transform parentTransform, string prefix)
    {

        List<Transform> childTransforms = new List<Transform>();
        //this is supercustom for pupeteering case
        //if (tar.useInPupeteering)
        //         childTransforms = parentTransform.GetComponentsInChildren<Transform>().Where(t => t.name == "K_" + mjBody.name).Where(x => x.transform.GetComponent<MjBody>() != null).ToList();
        // else
        
        childTransforms = parentTransform.GetComponentsInChildren<Transform>().Where(t => t.name == prefix + mjBody.name).ToList();

        
        if (childTransforms.Count() > 1)
        {
            Debug.LogWarning($"More than 1 match found for body {mjBody.name}: {string.Join(", ", childTransforms.Select(t => t.name))} Kinematic rig creation would likely fail.");
            return;
        }
        if (childTransforms.Count() < 1)
        {
            Debug.LogWarning($"No match found for body {mjBody.name}. The corresponding animated transform is expected to share the name of the MjBody, being the ref:" + prefix + mjBody.name );
            return;
        }

        MjFiniteDifferenceBody finiteDifferenceBody = childTransforms.First().gameObject.GetComponent<MjFiniteDifferenceBody>();
        if (finiteDifferenceBody == null)
        {
            finiteDifferenceBody = childTransforms.First().gameObject.AddComponent<MjFiniteDifferenceBody>();

        }


        finiteDifferenceBody.PairedBody = mjBody;

        foreach (var joint in mjBody.GetBodyChildComponents<MjBaseJoint>())
        {
            var finiteDifferenceJoint = new GameObject(prefix + joint.name).AddComponent<MjFiniteDifferenceJoint>();
            finiteDifferenceJoint.transform.SetLocalPositionAndRotation(joint.transform.localPosition, joint.transform.localRotation);
            finiteDifferenceJoint.transform.parent = finiteDifferenceBody.transform;
            finiteDifferenceJoint.PairedJoint = joint;
        }
        foreach (var childBody in mjBody.GetBodyChildComponents<MjBody>())
        {
            RecursiveComponentCreation(tar,childBody, finiteDifferenceBody.transform, prefix);
        }
    }
}
