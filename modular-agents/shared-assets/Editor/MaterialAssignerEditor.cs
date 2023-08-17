using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace ModularAgents.EditorScripts
{

[CustomEditor(typeof(MaterialAssigner))]
public class MaterialAssignerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        base.OnInspectorGUI();


        if (GUILayout.Button("Apply Material"))
        {
            var t = target as MaterialAssigner;
            t.SetMaterial();
        }


        serializedObject.ApplyModifiedProperties();

    }
}
}