using Mujoco;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class HomingTransform : MonoBehaviour
{
    [SerializeField]
    List<HomingTransformRange> controlledTransforms;

    [SerializeField]
    Transform endEffectorTransform;

    [SerializeField]
    Transform trackingTransform;

    [SerializeField]
    float smoothingFactor;


    float t;
    const float maxVal = 100000;

    // Update is called once per frame
    void FixedUpdate()
    {
        t += Time.fixedDeltaTime;
        if (t > maxVal) t = 0;

        foreach(var ht in controlledTransforms) 
        {
            ht.LerpToTransformState(ht.GetPerlin(t));
        }
        trackingTransform.position = Vector3.Lerp(endEffectorTransform.position, trackingTransform.position, smoothingFactor);
    }

    

    [Serializable]
    private class HomingTransformRange
    {
        [SerializeField]
        Transform transform;

        [SerializeField]
        Vector3 minPos;

        [SerializeField]
        Vector3 maxPos;

        [SerializeField]
        Vector3 minRot;

        [SerializeField]
        Vector3 maxRot;

        [SerializeField]
        Vector2 timeMultiplier;

        [SerializeField]
        Vector2 perlinClamp;

        public float GetPerlin(float t) 
        {
            return Mathf.Clamp(Mathf.PerlinNoise(timeMultiplier[0] * t, timeMultiplier[1] * t), perlinClamp[0], perlinClamp[1]);
        }

        public void LerpToTransformState(float val)
        {
            transform.localPosition = Vector3.LerpUnclamped(minPos, maxPos, val);
            transform.localRotation = Quaternion.Euler(Vector3.LerpUnclamped(minRot, maxRot, val));
        }

    }
}
