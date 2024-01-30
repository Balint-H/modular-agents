using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using ModularAgents.Kinematic;
using ModularAgents.Kinematic.Mujoco;
using ModularAgents.DeepMimic;
using System.Linq;
using static Google.Protobuf.WellKnownTypes.Field.Types;

public class ComparePoseRewards : MonoBehaviour
{
    [SerializeField]
    protected Transform pupetRoot;

    [SerializeField]
    protected Transform FDRoot;


    [SerializeField]
    protected Transform simRoot;


    [SerializeField]
    protected BodyChain pupetChain;
    [SerializeField]
    protected BodyChain FDChain;
    [SerializeField]
    protected BodyChain simChain;


    public float[] pupetAngles;
    public float pupetLoss;
    public float pupetReward;

    public float[] FDAngles;
    public float FDLoss;
    public float FDReward;
    public string[] FDNames;
    public string[] pupetNames;

    public Quaternion pupet11;
    public Quaternion FD11;

    // Start is called before the first frame update
    void Start()
    {

        simChain = simRoot.GetBodyChain();
        pupetChain = pupetRoot.GetBodyChain();
        FDChain = FDRoot.GetBodyChain();
    }
    protected void PoseReward()
    {
         pupet11 = pupetChain[11].LocalRotation;
         FD11 = FDChain[11].LocalRotation;

        float anglePupet10 = Quaternion.Angle(simChain[10].LocalRotation, pupetChain[10].LocalRotation) * Mathf.Deg2Rad;
        float anglePD10 = Quaternion.Angle(simChain[10].LocalRotation, FDChain[10].LocalRotation) * Mathf.Deg2Rad;



        pupetLoss = simChain.Zip(pupetChain, (sim, kin) => Sq(Quaternion.Angle(sim.LocalRotation, kin.LocalRotation) * Mathf.Deg2Rad)).Sum();
        pupetAngles = simChain.Zip(pupetChain, (sim, kin) => Sq(Quaternion.Angle(sim.LocalRotation, kin.LocalRotation) * Mathf.Deg2Rad)).ToArray();

        pupetNames = simChain.Select(x => x.Name).ToArray();

        pupetReward =  Mathf.Exp(-2f * pupetLoss / pupetChain.Count);

        FDLoss = simChain.Zip(FDChain, (sim, kin) => Sq(Quaternion.Angle(sim.LocalRotation, kin.LocalRotation) * Mathf.Deg2Rad)).Sum();
        FDAngles = simChain.Zip(FDChain, (sim, kin) => Sq(Quaternion.Angle(sim.LocalRotation, kin.LocalRotation) * Mathf.Deg2Rad)).ToArray();

        FDNames = simChain.Select( x => x.Name).ToArray();


        FDReward = Mathf.Exp(-2f * FDLoss / FDChain.Count);




    }
    // Update is called once per frame
    void Update()
    {
        PoseReward();
    }



    private static float Sq(float a) => a * a;
}
