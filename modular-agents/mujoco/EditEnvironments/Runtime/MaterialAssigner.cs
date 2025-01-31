using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MaterialAssigner : MonoBehaviour
{
    [SerializeField]
    Material material;

    [SerializeField]
    Transform root;

    public void SetMaterial()
    {
        foreach(var mr in root.GetComponentsInChildren<MeshRenderer>())
        {
            mr.material = material;
        }
    }
}

