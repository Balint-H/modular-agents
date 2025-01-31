using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.Assertions;

using System;
using Unity.MLAgents.Policies;
using ModularAgents.MotorControl;
using ModularAgents.Kinematic;

namespace ModularAgents.DReCon
{ 
/// <summary>
/// Deprecated, use ModularAgents with a SmoothedActuatorComponent.
/// </summary>
public class DReConAgent : Agent, IRememberPreviousActions, IEventsAgent
{
    [Header("Settings")]

    [SerializeField]
    private float fixedDeltaTime = 1f / 60f;
    [SerializeField]
    private float actionSmoothingBeta = 0.2f;
    [SerializeField]
    private int maxStep = 0;

    [SerializeField]
    GameObject kinematicRigObject;

    IKinematicReference kinematicRig;

    [SerializeField]
    ObservationSignal observationSignal;

    [SerializeField]
    RewardSignal rewardSignal;

    [SerializeField]
    DReConActuator ragDollMuscles;


    DecisionRequester decisionRequester;
    BehaviorParameters behaviorParameters;

    float[] previousActions;
    public float[] PreviousActions { get => previousActions;}



    public GameObject KinematicRigObject { get => kinematicRigObject; }



    protected bool hasLazyInitialized;

    public event EventHandler<AgentEventArgs> OnPostAction;
    public event EventHandler<AgentEventArgs> OnBegin;
    public event EventHandler<AgentEventArgs> OnStart;

    public float ObservationTimeDelta => fixedDeltaTime * decisionRequester.DecisionPeriod;

    public float ActionTimeDelta => decisionRequester.TakeActionsBetweenDecisions ? fixedDeltaTime : fixedDeltaTime * decisionRequester.DecisionPeriod;
    public float FixedDeltaTime { get => fixedDeltaTime; }

    public int RememberedActionSize => ragDollMuscles? ragDollMuscles.ActionSpaceSize : GetComponentsInChildren<ActuatorComponent>().Sum(ac => ac.ActionSpec.NumContinuousActions);


    public int ObservationSpaceSize => observationSignal.Size;

    public override void Initialize()
    {
        this.MaxStep = maxStep;
        Assert.IsFalse(hasLazyInitialized);
        hasLazyInitialized = true;
        if (fixedDeltaTime>0)
            Time.fixedDeltaTime = fixedDeltaTime;
        
        decisionRequester = GetComponent<DecisionRequester>();
    }



    public void Reset()
    {
        rewardSignal.OnAgentStart();
        observationSignal.OnAgentStart();

        if (kinematicRigObject != null)
        {
            kinematicRig = kinematicRigObject.GetComponent<IKinematicReference>();
            kinematicRig.OnAgentInitialize();
        }

        InitializeMuscles();

    }


    void InitializeMuscles()
    {

        if (ragDollMuscles == null) ragDollMuscles = GetComponent<DReConActuator>();
        ragDollMuscles.OnAgentInitialize(this);
        previousActions = ragDollMuscles.GetActionsFromState();
    }


    protected void Start()
    {
        rewardSignal.OnAgentStart();
        observationSignal.OnAgentStart();

        if (kinematicRigObject != null)
        {
            kinematicRig = kinematicRigObject.GetComponent<IKinematicReference>();
            kinematicRig.OnAgentInitialize();
        }

        if (ragDollMuscles) ragDollMuscles = ragDollMuscles.GetComponent<DReConActuator>();
        if (ragDollMuscles)
        {
            ragDollMuscles.OnAgentInitialize(this);
            previousActions = ragDollMuscles.GetActionsFromState();
        }
        else
        {
            previousActions = new float[RememberedActionSize];
        }

        OnStart?.Invoke(this, AgentEventArgs.Empty);
    }

    override public void CollectObservations(VectorSensor sensor)
    {
        Assert.IsTrue(hasLazyInitialized);

        observationSignal.PopulateObservations(sensor);
    }
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        Assert.IsTrue(hasLazyInitialized);
        float[] vectorAction = actionBuffers.ContinuousActions.ToArray();
        vectorAction = SmoothActions(vectorAction);
        
        if(ragDollMuscles)
            ragDollMuscles.ApplyActions(vectorAction, ActionTimeDelta);

        previousActions = vectorAction;

        float currentReward = rewardSignal.Reward;
        AddReward(currentReward);
        OnPostAction?.Invoke(this, new AgentEventArgs(vectorAction, currentReward));
    }
    public override void OnEpisodeBegin()
    {
        if (ragDollMuscles)
            previousActions = ragDollMuscles.GetActionsFromState();
        else
            previousActions = new float[previousActions.Length];
        OnBegin?.Invoke(this, AgentEventArgs.Empty);
    }



    float[] SmoothActions(float[] vectorAction)
    {
        var smoothedActions = vectorAction
            .Zip(PreviousActions, (a, y) => actionSmoothingBeta * a + (1f - actionSmoothingBeta) * y)
            .ToArray();
        return smoothedActions;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        float currentReward = rewardSignal.Reward;
        AddReward(currentReward);
    }

    public void SetPreviousActions(float[] vectorAction)
    {
        previousActions = vectorAction;
    }

   /* public override void CollectGailObservations(VectorSensor sensor)
    {
        sensor.AddObservation(false);
    }*/
}
}