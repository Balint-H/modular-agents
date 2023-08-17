using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;
using Mujoco;

namespace Mujoco
{ 


public class MjMeshExporter : EditorWindow
{
    [MenuItem("GameObject/Export Mujoco Mesh")]
    public static void ExportMeshes()
    {
        foreach (var meshfilter in Selection.activeGameObject.GetComponentsInChildren<MeshFilter>())
        {
            var meshPath = $"Assets/Resources/{meshfilter.name}.asset";
            AssetDatabase.CreateAsset(meshfilter.sharedMesh, meshPath);

            AssetDatabase.SaveAssets();

            SerializedObject meshFilterSerialized = new SerializedObject(meshfilter);
            SerializedProperty sharedMeshProp = meshFilterSerialized.FindProperty("m_Mesh");
            var myMesh = Resources.Load<Mesh>($"{meshfilter.name}");
            sharedMeshProp.objectReferenceInstanceIDValue = myMesh.GetInstanceID();

            meshFilterSerialized.ApplyModifiedProperties();

        }
    }

}
    }
