using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Hdf5DReConIntent : MonoBehaviour, IAnimationController
{
    [SerializeField]
    Hdf5Loader loader;

    public Vector3 GetDesiredVelocity()
    {
        var vel = loader.motionFiles[loader.CurClipIdx].RootVel(loader.CurFrameIndexInClip);
        return new Vector3(vel[0], vel[1], vel[2]);
    }

    public void OnReset()
    {

    }

}
