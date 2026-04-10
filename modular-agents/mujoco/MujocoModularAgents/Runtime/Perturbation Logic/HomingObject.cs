using Mujoco;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class HomingObject : MonoBehaviour
{
    [SerializeField] 
    MjActuator height;

    [SerializeField]
    MjActuator rot;

    [SerializeField]
    MjActuator dist;

    [SerializeField]
    List<float> timeMultipliers;

    float t;
    const float maxVal = 100000;

    // Update is called once per frame
    void Update()
    {
        t += Time.deltaTime;
        if (t > maxVal) t = 0;
        var rvs = new[] { Mathf.PerlinNoise(timeMultipliers[0] * t, 0), Mathf.PerlinNoise(0, timeMultipliers[1] * t), Mathf.PerlinNoise(-timeMultipliers[2] * t, 0) };
        foreach(var pair in rvs.Zip(new[] {height, rot, dist}, Tuple.Create)) 
        {
            pair.Item2.Control = LerpToControlRange(pair.Item2, pair.Item1);
        }

    }

    private float LerpToControlRange(MjActuator actuator, float val)
    {
        return Mathf.Lerp(actuator.CommonParams.CtrlRange[0], actuator.CommonParams.CtrlRange[1], val);
    }
}
