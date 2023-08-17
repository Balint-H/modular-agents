using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class UnityEventHandler : TrainingEventHandler
{
    [SerializeField]
    UnityEvent unityEvent;

    public override EventHandler Handler => (object sender, EventArgs e) => unityEvent.Invoke();
}
