using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using ModularAgents.Kinematic;
using System;

namespace ModularAgents.DReCon
{

    /// <summary>
    /// Keeps track of Collider components, their local bounding boxes, and the global points of the bounding box face centers for comparison with other chains.
    /// </summary>
    public class BoundingBoxChain : BodyChain
    {
        private IReadOnlyList<Collider> colliders;
        private IReadOnlyList<Bounds> bounds;

        private IReadOnlyList<BoundingPoints> points;

        private IEnumerable<Quaternion> LocalRotations { get => this.Select(k => k.LocalRotation); }


        public int ColliderCount { get => colliders.Count; }

        public BoundingBoxChain(BodyChain chain)
        {
            this.chain = GetValidKinematics(chain).ToList();
            SetupFromColliders();
            mass = this.chain.Select(k => k.Mass).Sum();
        }

        private void SetupFromColliders()
        {
            this.colliders = GetCollidersFromChain().ToList().AsReadOnly();
            bounds = colliders.Select(col => GetColliderBounds(col)).ToList().AsReadOnly();

            /*   chain = colliders.Select(col => col.attachedRigidbody != null ?
                                                   (IKinematic)new RigidbodyAdapter(col.attachedRigidbody) :
                                                   (col.attachedArticulationBody != null ?
                                                       new ArticulationBodyAdapter(col.attachedArticulationBody) : col.transform.parent.GetKinematic())).ToList().AsReadOnly();
            */
            points = chain.Zip(bounds, (bod, bound) => new BoundingPoints(bound, bod)).ToList().AsReadOnly();

            //Debug.Log(string.Join(", ", chain.Select(k => k.Name)));

        }

        protected IEnumerable<Collider> GetCollidersFromChain()
        {
            return chain.Select(k => k.gameObject.transform.GetComponentInDirectChildren<Collider>()).Where(c => c != null);
        }

        private IEnumerable<IKinematic> GetValidKinematics(IEnumerable<IKinematic> kinematics)
        {
            return kinematics.Where(k => k.gameObject.transform.GetComponentInDirectChildren<Collider>());
        }

        public static (float, float, float) CalulateDifferences(BoundingBoxChain chainA, ReferenceFrame referenceFrameA, BoundingBoxChain chainB, ReferenceFrame referenceFrameB)
        {
            var pointsA = chainA.points;
            var pointsB = chainB.points;
            float posDiff = pointsA.Zip(pointsB, (a, b) => BoundingPoints.PositionalDifference(a, referenceFrameA, b, referenceFrameB)).Sum();
            float velDiff = pointsA.Zip(pointsB, (a, b) => BoundingPoints.VelocityDifference(a, referenceFrameA, b, referenceFrameB)).Sum();

            float localPoseDiff = chainA.LocalRotations.Zip(chainB.LocalRotations, (ra, rb) => Quaternion.Angle(ra, rb) * Mathf.Deg2Rad).Sum();

            return (posDiff, velDiff, localPoseDiff);
        }

        /// <summary>
        /// Returns sum of square differences instead of the sum of differences. Not strictly according to DReCon paper, but closer to the earlier implementation
        /// </summary>
        public static (float, float, float) CalulateSquareDifferences(BoundingBoxChain chainA, ReferenceFrame referenceFrameA, BoundingBoxChain chainB, ReferenceFrame referenceFrameB)
        {
            var pointsA = chainA.points;
            var pointsB = chainB.points;
            float squarePosDiff = pointsA.Zip(pointsB, (a, b) => BoundingPoints.SquarePositionalDifference(a, referenceFrameA, b, referenceFrameB)).Sum();
            float squareVelDiff = pointsA.Zip(pointsB, (a, b) => BoundingPoints.SquareVelocityDifference(a, referenceFrameA, b, referenceFrameB)).Sum();

            float squareLocalPoseDiff = chainA.LocalRotations.Zip(chainB.LocalRotations, (ra, rb) => Quaternion.Angle(ra, rb) * Mathf.Deg2Rad).Select(x => x * x).Sum();

            return (squarePosDiff, squareVelDiff, squareLocalPoseDiff);
        }

        #region Axis aligned bounding box methods
        private Bounds GetColliderBounds(Collider collider)
        {
            switch (collider)
            {
                case BoxCollider b:
                    return GetBounds(b);
                case SphereCollider s:
                    return GetBounds(s);
                case CapsuleCollider c:
                    return GetBounds(c);
                default:
                    throw new NotImplementedException($"Collider type of \"{collider.name}\" not supported");
            }
        }

        private Bounds GetBounds(BoxCollider box)
        {
            return new Bounds(box.center, box.size);
        }

        private Bounds GetBounds(SphereCollider sphere)
        {
            float d = sphere.radius * 2f;
            return new Bounds(sphere.center, new Vector3(d, d, d));
        }

        private Bounds GetBounds(CapsuleCollider capsule)
        {
            float d = capsule.radius * 2f;
            float h = capsule.height;
            Vector3 size = Vector3.one * d;
            size[capsule.direction] = h;
            return new Bounds(capsule.center, size);
        }
        #endregion

        public void Draw()
        {
            foreach (var bps in points)
            {
                bps.Draw();
            }
        }

        public void DrawVelocities(ReferenceFrame referenceFrame, int bodyIndex, float scale = 0.1f)
        {
            var drawPos = points[bodyIndex].GetCharacterPoints(referenceFrame).Select(p => referenceFrame.CharacterToWorld(p));
            var drawVel = points[bodyIndex].GetCharacterVelocities(referenceFrame).Select(v => referenceFrame.CharacterDirectionToWorld(v));

            foreach ((var p, var v) in drawPos.Zip(drawVel, Tuple.Create))
            {
                Gizmos.DrawLine(p, p + v * scale);
            }
        }

        public static void DrawPositionalDifferences(BoundingBoxChain chainA, ReferenceFrame referenceFrameA, BoundingBoxChain chainB, ReferenceFrame referenceFrameB, int bodyIdx)
        {
            var pointsA = chainA.points[bodyIdx];
            var pointsB = chainB.points[bodyIdx];

            var distanceVectors = pointsA.GetCharacterPoints(referenceFrameA).Zip(pointsB.GetCharacterPoints(referenceFrameB), (a, b) => (a - b));


            foreach ((var distance, var chP) in distanceVectors.Zip(pointsB.GetCharacterPoints(referenceFrameB), Tuple.Create))
            {
                var p = referenceFrameB.CharacterToWorld(chP);
                var d = referenceFrameB.CharacterDirectionToWorld(distance);
                Gizmos.DrawLine(p, p + d);
            }

        }

        public static void DrawVelocityDifferences(BoundingBoxChain chainA, ReferenceFrame referenceFrameA, BoundingBoxChain chainB, ReferenceFrame referenceFrameB, int bodyIdx, float scale = 0.1f)
        {
            var drawPos = chainA.points[bodyIdx].GetCharacterPoints(referenceFrameA).Select(p => referenceFrameA.CharacterToWorld(p));

            var velDif = chainA.points[bodyIdx].GetCharacterVelocities(referenceFrameA).Zip(chainB.points[bodyIdx].GetCharacterVelocities(referenceFrameB), (a, b) => (a - b));

            foreach ((var p, var v) in drawPos.Zip(velDif, Tuple.Create))
            {
                Gizmos.DrawLine(p, p + v * scale);
            }
        }

        // TODO: A cleaner way to do this. In general a robust way to traverse transform chains without the need for names.
        private static IEnumerable<T> InNameOrder<T>(IEnumerable<T> toSort, IEnumerable<IKinematic> reference) where T : Component
        {
            return reference.Select(u => toSort.FirstOrDefault(t => Utils.SegmentName(t.name) == Utils.SegmentName(u.Name))).Where(t => t != null);
        }

        private static IEnumerable<Collider> GetCollidersFromChain(IReadOnlyList<IKinematic> chain)
        {
            return chain.Select(k => k.gameObject.transform.GetComponentInDirectChildren<Collider>()).Where(c => c != null);
        }

        protected override IReadOnlyList<IKinematic> GetKinematicChain(Transform root)
        {
            throw new NotImplementedException();
        }


        // Produce the 6 face center points
        private class BoundingPoints
        {
            private Vector3[] points;
            private Vector3 center;

            private IKinematic body;
            private Matrix4x4 LocalToWorldMatrix => body.TransformMatrix;

            public BoundingPoints(Bounds bounds, IKinematic body)
            {
                points = new Vector3[6];
                Vector3 c = bounds.center;

                //0: Right, 1: Left, 2: Top, 3: Bottom, 4: Front, 5: Back
                points[0] = new Vector3(bounds.max.x, c.y, c.z);
                points[1] = new Vector3(bounds.min.x, c.y, c.z);
                points[2] = new Vector3(c.x, bounds.max.y, c.z);
                points[3] = new Vector3(c.x, bounds.min.y, c.z);
                points[4] = new Vector3(c.x, c.y, bounds.max.z);
                points[5] = new Vector3(c.x, c.y, bounds.min.z);

                this.body = body;
                this.center = c;
            }

            public IEnumerable<Vector3> GetCharacterPoints(ReferenceFrame characterReferenceFrame)
            {

                Matrix4x4 localToCharacterMatrix = characterReferenceFrame.InverseMatrix * LocalToWorldMatrix; //Stored so to avoid calculating it every time
                return points.Select(p => localToCharacterMatrix.MultiplyPoint3x4(p));

            }

            public IEnumerable<Vector3> GetCharacterVelocities(ReferenceFrame characterReferenceFrame)
            {
                //Take angular velocity into account, alternatively: Vector3.Cross((worldPoint-worldCOM), worldAngularVelocity) + worldLinearVelocity;
                Matrix4x4 worldToCharacterMatrix = characterReferenceFrame.InverseMatrix;
                return points.Select(p => worldToCharacterMatrix.MultiplyVector(body.GetRelativePointVelocity(p)));
            }



            public static float PositionalDifference(BoundingPoints pointsA, ReferenceFrame referenceFrameA, BoundingPoints pointsB, ReferenceFrame referenceFrameB)
            {

                return pointsA.GetCharacterPoints(referenceFrameA).Zip(pointsB.GetCharacterPoints(referenceFrameB), (a, b) => (a - b).magnitude).Sum();
            }

            public static float VelocityDifference(BoundingPoints pointsA, ReferenceFrame referenceFrameA, BoundingPoints pointsB, ReferenceFrame referenceFrameB)
            {
                return pointsA.GetCharacterVelocities(referenceFrameA).Zip(pointsB.GetCharacterVelocities(referenceFrameB), (a, b) => (a - b).magnitude).Sum();
            }

            public static float SquarePositionalDifference(BoundingPoints pointsA, ReferenceFrame referenceFrameA, BoundingPoints pointsB, ReferenceFrame referenceFrameB)
            {

                return pointsA.GetCharacterPoints(referenceFrameA).Zip(pointsB.GetCharacterPoints(referenceFrameB), (a, b) => (a - b).magnitude).Select(x => x * x).Sum();
            }

            public static float SquareVelocityDifference(BoundingPoints pointsA, ReferenceFrame referenceFrameA, BoundingPoints pointsB, ReferenceFrame referenceFrameB)
            {
                return pointsA.GetCharacterVelocities(referenceFrameA).Zip(pointsB.GetCharacterVelocities(referenceFrameB), (a, b) => (a - b).magnitude).Select(x => x * x).Sum();
            }

            public void Draw()
            {
                Matrix4x4 localToWorldMatrix = LocalToWorldMatrix;
                var worldPoints = points.Select(p => localToWorldMatrix.MultiplyPoint3x4(p));
                Gizmos.color = Color.white;
                foreach (var p in worldPoints)
                {
                    Gizmos.DrawSphere(p, 0.01f);
                }
            }

            public override string ToString()
            {
                return body.Name;
            }
        }

    }
}




