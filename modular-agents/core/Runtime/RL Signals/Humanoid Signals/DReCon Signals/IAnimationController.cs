using UnityEngine;


public interface IAnimationController
{
    void OnReset();
    Vector3 GetDesiredVelocity();
}