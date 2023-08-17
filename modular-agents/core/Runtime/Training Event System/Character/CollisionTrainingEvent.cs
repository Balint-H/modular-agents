using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;


/// <summary>
/// This Event will be triggered when the GameObject that it is attached to receives a collision. 
/// This works in PhysX and needs a Collider attached to it.
/// </summary>
public class CollisionTrainingEvent : TrainingEvent
{


    [Tooltip("If left empty any collision with this GameObject will trigger this event.")]
    [SerializeField]
    List<GameObject> inclusionFilter;

    private void OnCollisionEnter(Collision collision)
    {
        if (inclusionFilter.Count==0 ||  inclusionFilter.Any(go => go == collision.gameObject)) OnTrainingEvent(System.EventArgs.Empty);
    }
}
