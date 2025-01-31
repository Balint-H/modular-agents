using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class MjMocapPlaybackEvent : TrainingEvent
{
    [SerializeField]
    Hdf5Loader loader;

    [SerializeField, Tooltip("This event will be triggered at the end of the clip with this index")]
    int clipIdxEnded;



    protected override void InitializeEvent()
    {
        loader.ClipEndedEvent += (_, args) => { if (args.clipId == clipIdxEnded) OnTrainingEvent(System.EventArgs.Empty); };
    }
}