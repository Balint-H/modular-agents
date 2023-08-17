using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Only invokes the wrapped handler every "n_i"-th call, where "n_i" is cycled through a list of values
/// </summary>
public class CounterWrapHandler : TrainingEventHandler
{
    public override EventHandler Handler => (object sender, EventArgs e) => CountDown();

    [SerializeField] 
    TrainingEventHandler wrappedHandler;

    [SerializeField, Tooltip("Use \"0\" if no delay desired")]
    List<int> delayCounts;

    [SerializeField]
    int curRemainingDelay;
    int curIndex;

    [SerializeField]
    CounterSelectionMode counterSelectionMode;

    [Serializable]
    private enum CounterSelectionMode
    {
        Random,
        Ordered
    }

    private void Awake()
    {
        curIndex = -1; // So in ordered mode we start with the first element
        UpdateToNextCountdown();
    }

    void UpdateToNextCountdown()
    {
        switch(counterSelectionMode)
        {
            case CounterSelectionMode.Ordered:
                curIndex = ++curIndex % delayCounts.Count;
                break;
            case CounterSelectionMode.Random:
                curIndex = UnityEngine.Random.Range(0, delayCounts.Count);
                break;
        }
        curRemainingDelay = delayCounts[curIndex];
    }

    // Start is called before the first frame update
    void CountDown()
    {
        if (curRemainingDelay-- == 0)
        {
            wrappedHandler.Handler?.Invoke(this, EventArgs.Empty);
            UpdateToNextCountdown();
        }
    }
}
