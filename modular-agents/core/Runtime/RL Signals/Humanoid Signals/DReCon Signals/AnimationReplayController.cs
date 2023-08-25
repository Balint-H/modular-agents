using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AnimationReplayController : MonoBehaviour, IAnimationController
{

    Animator _anim;

    public void Start()
    {
        if (!_anim)
            _anim = GetComponent<Animator>();
        if (!_anim)
            Debug.LogWarning("No animator component found. Will always return zero desired velocity.");
        if (_anim && (!_anim.applyRootMotion || !_anim.hasRootMotion))
            Debug.LogWarning("Animator found, but no root motion in animator.");
    }

    public void OnReset()
    {
    }

    public Vector3 GetDesiredVelocity()
    {
        return _anim? _anim.velocity : Vector3.zero;
    }
}
