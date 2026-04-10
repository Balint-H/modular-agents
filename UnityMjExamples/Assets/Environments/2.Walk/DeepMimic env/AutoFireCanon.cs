using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AutoFireCanon : MonoBehaviour
{

    public float wait2start = 0;

    public float fireEveryMinSecs = 1, fireEveryMaxSecs=2;

    ManualEvent manualEvent;


    bool shootingStarted = false;

    void Start()
    {
        manualEvent = GetComponent<ManualEvent>();


    }

    private void Update()
    {
        if ((wait2start < Time.time) &!shootingStarted)
        {
            shootingStarted=true;

            StartCoroutine(fireLoop());
        }
            
                
                
     }


    IEnumerator fireLoop() 
    {
        for(; ; )
        {
            manualEvent.ManuallyTrigger(System.EventArgs.Empty);
            Debug.Log("autofirecanon in action");
            yield return new WaitForSeconds(Random.Range(fireEveryMinSecs, fireEveryMaxSecs) );
        }
    
    }
}

