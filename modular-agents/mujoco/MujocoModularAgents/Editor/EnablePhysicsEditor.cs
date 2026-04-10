using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(EnablePhysics))]
public class EnablePhysicsEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();


        base.OnInspectorGUI();


        if (GUILayout.Button("Enable/Disable Manually"))
        {
            EnablePhysics t = target as EnablePhysics;

            t.Switch();
          
        }


        serializedObject.ApplyModifiedProperties();

    }
}
