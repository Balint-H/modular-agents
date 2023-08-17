using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AnimationTrainingEvent : TrainingEvent
{
    [SerializeField] Animator animator;
    [SerializeField] int clipId;
    [SerializeField] float time;

    // Start is called before the first frame update
    void Awake()
    {
        AnimationClip clip = animator.runtimeAnimatorController.animationClips[clipId];

        var animEvent = new AnimationEvent();
        animEvent.functionName = "AnimEventWrapper";
        animEvent.time = time;

        clip.AddEvent(animEvent);
    }

   void AnimEventWrapper()
    {
        OnTrainingEvent(System.EventArgs.Empty);
    }
}
