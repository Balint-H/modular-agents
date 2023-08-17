using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class PauseEditorHandler : TrainingEventHandler
{
    public override EventHandler Handler => (object _, EventArgs _) => EditorApplication.isPaused = true;

}
