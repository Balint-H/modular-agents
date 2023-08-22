using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ModularAgents.Throwing
{
    public class TargetHandler : TrainingEventHandler
    {
      

        [SerializeField]
        bool shouldTrackHeight;

      
        public bool h;

     
        [SerializeField]
        Vector3 idealPosition;

        [SerializeField]
        Transform pivot;
        Matrix4x4 pivotMatrix;

      

        [SerializeField]
        Transform currentTarget;

        [SerializeField]
        float radiusRange = 0.5f;

        [SerializeField]
        float angleRange = 0.1f;

        [SerializeField]
        float heightRange = 0.125f;

        public Vector3 CurrentTargetPosition{ get => currentTarget.position; }


    public override EventHandler Handler => (sender, args) => ResetTarget();


    private void Awake()
    {
        pivotMatrix = pivot.localToWorldMatrix;
        h = false;
    }






    void ResetTarget()
    {
        h = false;
     
        Vector3 relTargetPos = pivotMatrix.inverse.MultiplyPoint3x4(idealPosition);
        float meanRadius = relTargetPos.Horizontal3D().magnitude;
        float meanAngle = Mathf.Deg2Rad * Vector3.Angle(Vector3.right, relTargetPos.Horizontal3D());
        float meanHeight = relTargetPos.y;

        

        float sampledRadius = UnityEngine.Random.Range(meanRadius - radiusRange, meanRadius + radiusRange);
        float sampledAngle = UnityEngine.Random.Range(meanAngle - angleRange, meanAngle + angleRange);
        float sampledHeight = UnityEngine.Random.Range(meanHeight - heightRange, meanHeight + heightRange);


        Vector3 sampledPosition = pivotMatrix.MultiplyPoint3x4(new Vector3(Mathf.Cos(sampledAngle) * sampledRadius, sampledHeight, Mathf.Sin(sampledAngle) * sampledRadius));

        currentTarget.position = sampledPosition;
    }

    private void OnCollisionEnter(Collision collision)
    {
        h = true;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

  
}
}