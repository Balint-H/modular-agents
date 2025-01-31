using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;


[CustomEditor(typeof(BodyNameMatcher))]
public class BodyNameMatcherEditor : Editor
{

    public override void OnInspectorGUI()
    {
        serializedObject.Update();


        base.OnInspectorGUI();

        if (GUILayout.Button("RenameBodesToMatchClosest"))
        {
            BodyNameMatcher t = target as BodyNameMatcher;

            t.RenameBodiesToMatchClosest();


        }


        if (GUILayout.Button("RestoreNames"))
        {
            BodyNameMatcher t = target as BodyNameMatcher;

            t.RestoreNames();


        }


        serializedObject.ApplyModifiedProperties();
    }


    }
