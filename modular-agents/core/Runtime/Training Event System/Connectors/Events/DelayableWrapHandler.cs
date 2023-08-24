using System;
using System.Collections;
using UnityEngine;


/// <summary>
/// Invokes the wrapped handler after a delay of the specified number of FixedUpdates.
/// </summary>
public class DelayableEventHandler: TrainingEventHandler
{
    [SerializeField]
    protected int framesToWait;

    [SerializeField] bool allowConcurrent = false;

    [SerializeField]
    private bool isWaiting;

    [SerializeField]
    TrainingEventHandler wrappedHandler;

    public bool IsWaiting { get => isWaiting; set => isWaiting = value; }

    public override EventHandler Handler => (object sender, EventArgs e) =>
    {
        if (!allowConcurrent && isWaiting)
            return;
            

        StartCoroutine(DelayedExecution(sender, e));
    };

    protected IEnumerator DelayedExecution(object sender, EventArgs args)
	{
        isWaiting = true;
        yield return WaitFrames();
        wrappedHandler.Handler?.Invoke(sender, args);	
        isWaiting = false;
	}

    protected IEnumerator WaitFrames()
    {
        for (int i = 0; i < framesToWait+1; i++) yield return new WaitForFixedUpdate();
    }
}