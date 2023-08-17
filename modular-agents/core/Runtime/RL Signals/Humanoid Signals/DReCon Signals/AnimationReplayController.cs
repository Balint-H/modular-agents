using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AnimationReplayController : MonoBehaviour, IAnimationController
{

    Animator _anim;

    public void OnEnable()
    {
        OnAgentInitialize();
    }

    public void OnAgentInitialize()
    {
        if (!_anim)
            _anim = GetComponent<Animator>();
    }

    public void OnReset()
    {
    }

    public Vector3 GetDesiredVelocity()
    {
        return _anim? _anim.velocity : Vector3.zero;
    }
}
