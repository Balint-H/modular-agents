using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PrintMessageHandler : TrainingEventHandler
{
    [SerializeField]
    string message;

    public override EventHandler Handler => (_, _) => Debug.Log(message);
}
