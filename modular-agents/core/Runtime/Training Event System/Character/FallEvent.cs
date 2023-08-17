using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;



namespace ModularAgents.TrainingEvents
{
    /// <summary>
    /// The fall event will be triggered when SimulationHead is farther than maxDistance from the KinematicHead. The useHeightOnly option will only consider the difference in height.
    /// </summary>


    public class FallEvent : TrainingEvent
    {

        [SerializeField]
        private Transform KinematicHead;

        [SerializeField]
        private Transform SimulationHead;

        [SerializeField]
        private bool useHeightOnly;

        [SerializeField]
        float maxDistance;


        private void FixedUpdate()
        {
            CheckFall();
        }

        private void CheckFall()
        {
            if (useHeightOnly ? Mathf.Abs(KinematicHead.position.y - SimulationHead.position.y) > maxDistance
                : (KinematicHead.position - SimulationHead.position).magnitude > maxDistance)
            {
                OnTrainingEvent(EventArgs.Empty);
            }
        }
    }
}