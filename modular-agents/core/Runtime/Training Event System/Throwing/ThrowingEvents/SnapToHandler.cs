using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace ModularAgents.Throwing
{ 
public class SnapToHandler : TrainingEventHandler
{
    [SerializeField]
    private Transform movedTransform;

    private ITeleportable teleportable;

    [SerializeField]
    private Transform targetedTransform;

    private void Awake()
    {
        var rb = movedTransform.GetComponent<Rigidbody>();

        if (rb != null)
        {
            teleportable = new RigidbodyAdaptor(rb);
        }
        else
        {
            teleportable = new TransformAdapter(movedTransform);
        }
    }

    private void SnapHandle(object sender, EventArgs eventArgs)
    {
        teleportable.Position = targetedTransform.position;
        teleportable.Rotation = targetedTransform.rotation;
    }

    public override EventHandler Handler => SnapHandle;

    interface ITeleportable
    {
        public Vector3 Position { get; set; }
        public Quaternion Rotation { get; set; }
    }

    //Only transform and Rigidbody supported, ArticulationBody doesn't play nice with snapping
    class TransformAdapter : ITeleportable
    {
        private Transform transform;

        public TransformAdapter(Transform transform)
        {
            this.transform = transform;
        }

        public Vector3 Position { get => transform.position; set => transform.position = value; }
        public Quaternion Rotation { get => transform.rotation; set => transform.rotation = value; }
    }

    class RigidbodyAdaptor : ITeleportable
    {
        private Rigidbody rigidbody;

        public RigidbodyAdaptor(Rigidbody rigidbody)
        {
            this.rigidbody = rigidbody;
        }

        public Vector3 Position { get => rigidbody.position; set => rigidbody.position = value; }
        public Quaternion Rotation { get => rigidbody.rotation; set => rigidbody.rotation = value; }
    }
}
}