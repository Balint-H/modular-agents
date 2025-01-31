using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Hdf5ClipEndedEvent : TrainingEvent
{
    [SerializeField]
    Hdf5Loader loader;

    private void Awake()
    {
        loader.ClipEndedEvent += (_, e) => OnTrainingEvent(e);
    }
}
