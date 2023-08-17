using UnityEngine;


public interface IAnimationController
{
    void OnAgentInitialize();
    void OnReset();
    Vector3 GetDesiredVelocity();
}