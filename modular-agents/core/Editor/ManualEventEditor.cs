using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(ManualEvent))]
public class ManualEventEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();


        base.OnInspectorGUI();


        if (GUILayout.Button("Manually Trigger"))
        {
            ManualEvent t = target as ManualEvent;

            t.ManuallyTrigger(System.EventArgs.Empty);
        }



        serializedObject.ApplyModifiedProperties();

    }
}
