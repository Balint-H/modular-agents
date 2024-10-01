using ModularAgents.DeepMimic;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Hdf5DeepMimicObservations : MjDeepMimicObservations
{
    [SerializeField]
    Hdf5Loader loader;


    protected override float GetPhase()
    {
        return loader.NormalizedTimeInClip;
    }
}
