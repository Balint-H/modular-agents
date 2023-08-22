using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Animator))]
public class AnimationTrainingEvent : TrainingEvent
{
//    [SerializeField] string clipName;
    [SerializeField] int clipId;
    [SerializeField] float time;

    // Start is called before the first frame update
    void Awake()
    {
  //      GetComponent<Animator>()
        AnimationClip clip = GetComponent<Animator>().runtimeAnimatorController.animationClips[clipId];

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
