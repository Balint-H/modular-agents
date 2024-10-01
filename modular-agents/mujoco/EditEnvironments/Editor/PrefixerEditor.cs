using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(Prefixer))]
public class PrefixerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();


        GUILayout.Label("");




        base.OnInspectorGUI();


        if (GUILayout.Button("Apply Prefix"))
        {
            Prefixer t = target as Prefixer;
            foreach (var a in t.prefixRoot.GetComponentsInChildren<Transform>())
            {
                a.name = a.name.TrimStart(t.prefixToCut.ToCharArray());
                a.name = t.prefix + a.name;
            }
        }



        serializedObject.ApplyModifiedProperties();

    }
}
