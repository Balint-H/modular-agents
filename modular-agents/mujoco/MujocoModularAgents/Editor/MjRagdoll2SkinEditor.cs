using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(MjRagdoll2Skin))]
public class MjRagdoll2SkinEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();


        base.OnInspectorGUI();


      /*
        if (GUILayout.Button("Print Ragdoll Mecanim Bones"))
        {
            MjRagdoll2Skin t = target as MjRagdoll2Skin;

            t.PrintRagdollAvatarNames();


        }


        if (GUILayout.Button("Print Skin Mecanim Bones"))
        {
            MjRagdoll2Skin t = target as MjRagdoll2Skin;

            t.PrintSkinSkeletonAvatarNames();


        }
    */

        if (GUILayout.Button("Initialize mapping"))
        {



            MjRagdoll2Skin t = target as MjRagdoll2Skin;

            InitRots ir = t.InEditorInitialize();
            AssetDatabase.CreateAsset(ir, "Assets/InitialRotations.asset");

        }


        serializedObject.ApplyModifiedProperties();

    }
}
