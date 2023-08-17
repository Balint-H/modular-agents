using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using ModularAgents.DReCon;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace ModularAgents.Legacy
{ 
public class PhysXPausableEnvironmentStepHandler : PausableEnvironmentStepHandler
{
    //This class  is a modification of PausableEnvironmentStepHandler to also include freezing the physics simulation in unity

    // [SerializeField]
    // Animator animator;

    [SerializeField]
    JumpToClipHandler jumpToClipHandler;

    [SerializeField]
    DReConAgent agentWithMuscles;

    ArticulationBody[] abs;

    bool firstFrame = true;

    public override EventHandler Handler => (object sender, EventArgs e) =>
    {
/*        if (framesToWait <= 0)
        {
          
        
            return;
        }
        framesToWait--;
*/
    };

    private void Awake()
    {
        Academy.Instance.AutomaticSteppingEnabled = false;
        Physics.simulationMode = SimulationMode.Script;
      //  abs = agentWithMuscles.GetComponentsInChildren<ArticulationBody>();
    }

  

    private void FixedUpdate()
    {
        if (framesToWait == 0)
        {

            /*
            foreach (ArticulationBody body in abs)
            {
                body.enabled = true;

            }*/


            //Academy.Instance.EnvironmentStep();
            //Physics.Simulate(Time.fixedDeltaTime);
            
            framesToWait--;



          
            firstFrame = true;
          


        }
        else if (framesToWait < 0) 
        {
            Academy.Instance.EnvironmentStep();
            Physics.Simulate(Time.fixedDeltaTime);
        }
        else
        {


            if (firstFrame) {




                agentWithMuscles.EpisodeInterrupted();
                agentWithMuscles.Reset();
                Academy.Instance.EnvironmentStep();
                Physics.Simulate(Time.fixedDeltaTime);
                /*
                foreach (ArticulationBody body in abs)
                {
                    body.enabled = false;

                }*/
                firstFrame = false;
#if UNITY_EDITOR
                EditorApplication.isPaused = true;
#endif

            }


            framesToWait--;
            if (jumpToClipHandler)
                jumpToClipHandler.PlayAtJump();//it places the animation at the right time until the setup can run again

           
            //if(agentWithMuscles != null)
            //   
        }

    }

}
}