using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HorizontalFollow : MonoBehaviour
{
    // Start is called before the first frame update
    [SerializeField]
    Transform trackedTransform;

    [SerializeField]
    Transform movedTransform;

    [SerializeField]
    float smoothingFactor;

    private void FixedUpdate()
    {
        float y = movedTransform.position.y;

        var smoothedPos = Vector3.Lerp(trackedTransform.position, movedTransform.position, smoothingFactor);
        smoothedPos.y = y;
        movedTransform.position = smoothedPos; 
    }
}
