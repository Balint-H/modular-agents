using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class PauseEditorHandler : TrainingEventHandler
{
    // public override EventHandler Handler => throw new NotImplementedException();
    public override EventHandler Handler => PauseInEditor;


    void PauseInEditor(object sender, EventArgs eventArgs)
    {

#if UNITY_EDITOR


        EditorApplication.isPaused = true;
#endif

    }


}
