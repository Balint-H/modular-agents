using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

namespace Mujoco.Extensions
{
    public static class Reposition
    {
        /// <summary>
        /// Rotates the segment's parent body so the child is as close as possible in global space to the target position (with minimal rotation).
        /// Must perform it proximal -> distal for this reason.
        /// </summary>
        ///  <param name="moveUsingParentRotation">If true, and parent body exists, then will perform the alignment by rotating the parent segment.
        /// Otherwise, will simply move the child body in the parent's frame. This leaves all the geoms, joints, inertials, etc. of the parent still 
        /// oriented the same way as they were. Set to false in branching segments, e.g. root, thorax, etc. </param>
        ///  <param name="originalJointOrientations">Original global joint orientations before any alignment, indexed by the joint game object instance ID.
        /// If provided, the joint's will be repositioned so the axes match the original ones. Ignored if moveUsingParentRotation is false.</param>
        public static void AlignBodyPosition(MjBaseBody body, Vector3 targetPos, bool moveUsingParentRotation=true, Dictionary<int, Quaternion> originalJointOrientations=null)
        {
            
            var parentBody = body.transform.parent.GetComponentInParent<MjBaseBody>();

            if (parentBody == null || !parentBody.GetComponentInParent<MjBaseBody>() || !moveUsingParentRotation)
            {
                
                body.transform.position = targetPos;
                return;
            }
            
            var childJoints = parentBody.GetBodyChildComponents<MjBaseJoint>().ToList();
            var fromTo = Quaternion.FromToRotation((body.transform.position - parentBody.transform.position), (targetPos - parentBody.transform.position));
            parentBody.transform.rotation = fromTo*parentBody.transform.rotation;
            if(originalJointOrientations == null) return;
            foreach (var j in childJoints)
            {
                j.transform.rotation = originalJointOrientations[j.GetInstanceID()];
            }
        }

        /// <summary>
        /// Swap the orientation of the body frames, so they match the provided reference. The child bodies are returned to their original global positions.
        /// </summary>
        public static void ReorientBodyFrame(MjBaseBody body, Quaternion targetOrientation)
        {
            var directChildren = body.transform.GetComponentsInDirectChildren<Transform>().ToList();
            var origRots = new Dictionary<int, (Vector3, Quaternion)>(directChildren
                .Select(t =>new KeyValuePair<int, (Vector3, Quaternion)>(t.GetInstanceID(), (t.position, t.rotation))).ToList());
            body.transform.rotation = targetOrientation;
            foreach (Transform directChild in directChildren)
            {
                var (pos, rot) = origRots[directChild.GetInstanceID()];
                directChild.rotation = rot;
                directChild.position = pos;
            }
        }
    }
}