using JetBrains.Annotations;
using MathNet.Numerics.Statistics;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Unity.VisualScripting;
using UnityEngine;

namespace Mujoco.Extensions
{
    /// <summary>
    /// Functions for scaling a MuJoCo humanoid to match a Unity animation skeleton.
    /// Assumptions:
    /// 1. All joints of a body are coincident.
    /// 2. Capsules, boxes, cylinders are all aligned with the longitudinal segment axis, or with the orthogonal plane.
    /// 3. We also only scale longitudinally and uniformly in orthogonal directions (joint angles and anatomical pose should not be affected by scaling).
    /// 4. Joints are coincident with body frames (preliminary implementation to handle violations is in place but not tested, a warning is issued if used).
    /// 5. End effectors have a site marking the end of their segment.
    /// 6. Assume that branching bodies are only in the torso and aligned with the global vertical direction.
    /// We don't use Unity transform scales, due to limitations in non-aligned scaling directions and rotating child bodies in scaled hierarchies.
    /// </summary>
    public static class Scaling
    {
        /// <summary>
        /// We define a ScalingSegment as the spatial connection between a joint that couples two bodies, and the joint(s) of the proximal body.
        /// A ScalingSegment corresponds to a body segment (e.g. thigh, shank), with a cylindrical representation aligned with the two joints.
        /// For this reason we assume that if there are multiple joints in a body, then they are coincident.
        /// This is closer to the way animation rigs are structured, and helps getting the correct axes and magnitude of scaling necessary.
        /// </summary>
        internal class ScalingSegment
        {
            readonly MjBaseJoint startJoint;

            readonly Transform startTransform;
            public Vector3 StartPoint => startTransform.position;

            
            readonly MjBaseJoint endJoint; // may or may not be defined
            readonly Transform endTransform;
            public Vector3 EndPoint => endTransform.position;

            public readonly MjBaseBody segmentBody;  // The body whose joint startJoint is.
            public readonly IReadOnlyList<MjBaseBody> segmentBodies;  // Inclusive of of segmentBody, also includes any fixed bodies connecting startJoint with endJoint.

            public IEnumerable<Vector3> segmentScalingDirections
            {
                get
                {
                    foreach ((var b1, var b2) in segmentBodies.SkipLast(1).Zip(segmentBodies.Skip(1), Tuple.Create))
                    {
                        if (b1.GetBodyChildComponents<MjBaseBody>().Count() > 1) yield return Vector3.up;
                        else yield return BodySegmentLengthVector(b1, b2).normalized;  // ASSUMPTION 6
                    }
                    yield return BodySegmentLengthVector(segmentBodies[segmentBodies.Count-1], childBody).normalized;
                }
            }
            public readonly MjBaseBody childBody;

            // Every scaling segment will have a start joint, not every joint starts a scaling segment.
            public MjBaseJoint StartJoint { get => startJoint; }

            public string MecanimName { get; set; }
            public string ChildMecanimName { get; set; }

            ScalingSegment parentSegment;
            public ScalingSegment ParentSegment
            {
                get => parentSegment;
                set
                {
                    parentSegment = value;
                    parentSegment.ChildSegments.Add(this);
                }
            }

            public List<ScalingSegment> ChildSegments { get; } = new List<ScalingSegment>();

            public IEnumerable<ScalingSegment> SubtreeSegments
            {
                get
                {
                    yield return this;
                    foreach (var ch in ChildSegments)
                    {
                        foreach(var s in ch.SubtreeSegments)
                        {
                            yield return s;
                        }
                    }
                    
                }
            }

            public float GeomAlignment
            {
                get
                {
                    List<float> results = new List<float>();
                    foreach ((var body, var dir) in segmentBodies.Zip(segmentScalingDirections, Tuple.Create))
                    {
                        foreach(var geom in body.GetBodyChildComponents<MjGeom>())
                        {
                            var alignments = new List<Vector3>() { geom.transform.right, geom.transform.up, geom.transform.forward }
                                                         .Select(v => dir.magnitude * Mathf.Abs(Vector3.Dot(v, dir))).ToList();
                            var orthogonalAlginments = alignments.Select(a => dir.magnitude*( 1 - a)).ToList();
                            alignments.AddRange(orthogonalAlginments);
                            results.Add(alignments.Max());
                        }
                    }
                    return (float) results.Mean();
                }
            }

            public Vector3 LongitudinalVector => (EndPoint - StartPoint).normalized;
            public float SegmentLength => (EndPoint - StartPoint).magnitude;


            // Parameters segmentStartBody and childBody don't need to be directly nested, can envelop sub kinematic chain.
            public ScalingSegment(MjBaseBody segmentStartBody, MjBaseBody childBody)
            {
                endJoint = childBody.GetBodyChildComponents<MjBaseJoint>().FirstOrDefault();  // ASSUMPTION 1
                startJoint = segmentStartBody.GetBodyChildComponents<MjBaseJoint>().FirstOrDefault();

                startTransform = startJoint ? startJoint.transform : segmentStartBody.transform;
                endTransform = endJoint ? endJoint.transform : childBody.transform;

                segmentBody = segmentStartBody;

                var segmentBodies = new List<MjBaseBody>() { segmentBody };
                segmentBodies.AddRange(endTransform.GetComponentsInParent<MjBaseBody>()
                                                   .TakeWhile(mjb => mjb != segmentBody)
                                                   .Skip(1)  // Need to skip as would include childBody as well
                                                   .Reverse());
                this.segmentBodies = segmentBodies;
                this.childBody = childBody;

            }

            public ScalingSegment(MjBaseBody segmentStartBody, MjSite endEffectorSite)
            {
                startJoint = segmentStartBody.GetBodyChildComponents<MjBaseJoint>().FirstOrDefault();  // ASSUMPTION 1
                startTransform = startJoint ? startJoint.transform : segmentStartBody.transform;
                segmentBody = segmentStartBody;

                
                endTransform = endEffectorSite.transform;

                var segmentBodies = new List<MjBaseBody>() { segmentBody };
                segmentBodies.AddRange(endEffectorSite.GetComponentsInParent<MjBaseBody>()
                                                      .TakeWhile(mjb => mjb != segmentBody)
                                                      .Reverse());
                this.segmentBodies = segmentBodies;
                this.childBody = null;
            }

            public Vector2 AppliedScale { get; set; }


            public IReadOnlyList<ComponentScaleCandidate> GetComponentScalingCandidates(float longitudinalScale, float orthogonalScale = 1) => EnumerateComponentScalingCandidates(longitudinalScale, orthogonalScale).ToList();

            /// <summary>
            /// Prepare a body and the MjComponents that belong to it for scaling, and return objects that can perform the operation. Also record how well aligned the scaling is with the geoms of the body.
            /// Based on this we may decide if we want to execute the operation or not.
            /// </summary>
            /// <param name="longitudinalScale">The scaling along the direction of the segment (lengthwise). </param>
            /// <param name="orthogonalScale">Width and depth scaling (thickness of the segment). </param>
            private IEnumerable<ComponentScaleCandidate> EnumerateComponentScalingCandidates(float longitudinalScale, float orthogonalScale = 1)
            {
                AppliedScale = new Vector2(longitudinalScale, orthogonalScale);
                foreach ((var body, var dir) in segmentBodies.Zip(segmentScalingDirections, Tuple.Create))
                {

                    yield return new ComponentScaleCandidate(body, longitudinalScale, orthogonalScale, GetGeomAlignment(body, dir), dir, this);

                }
            }

            public class ComponentScaleCandidate
            {
                public readonly MjBaseBody body;
                public readonly float longitudinalScale;
                public readonly float orthogonalScale;
                public readonly float geomAlignment;
                public readonly Vector3 globalScalingDirection;
                public readonly ScalingSegment segment;

                public ComponentScaleCandidate(MjBaseBody component, float longitudinalScale, float orthogonalScale, float geomAlignment, Vector3 globalScalingDirection, ScalingSegment segment)
                {
                    this.body = component;
                    this.longitudinalScale = longitudinalScale;
                    this.orthogonalScale = orthogonalScale;
                    this.geomAlignment = geomAlignment;
                    this.globalScalingDirection = globalScalingDirection;
                    this.segment = segment;
                }

                public void Scale(bool attemptMisalignedScale = false, GeomScalingOptions geomOptions=null)
                {
                    geomOptions ??= new GeomScalingOptions(attemptMisalignedScale: attemptMisalignedScale);

                    var globalParentBodyPosition = body.transform.position;
                    if (globalScalingDirection.magnitude == 0) return;
                    foreach (var segmentComponent in body.GetBodyChildComponents<MjComponent>())
                    {
                        if (segmentComponent == body) continue;
                        if (segmentComponent is MjBaseJoint && segmentComponent.transform.localPosition != Vector3.zero)
                        {
                            Debug.LogWarning("Scaling bodies with joints that do not coincide with body frames has not been tested. Ensure scaling is done correctly!");
                        }
                        segmentComponent.Scale(globalParentBodyPosition, globalScalingDirection, longitudinalScale, orthogonalScale, geomOptions);
                    }
                }

                public static ComponentScaleCandidate WeightedMix(IEnumerable<ComponentScaleCandidate> candidates)
                {
                    candidates = candidates.ToList();
                    var alignmentSum = candidates.Sum(c => c.geomAlignment);
                    float mixedLongitudinalScale = candidates.Sum(c => c.geomAlignment * c.longitudinalScale) / alignmentSum;
                    float mixedOrthogonalScale = candidates.Sum(c => c.geomAlignment * c.orthogonalScale) / alignmentSum;
                    float mixedAlignment = candidates.Average(c => c.geomAlignment);
                    Vector3 mixedScalingDirection = (candidates.Aggregate<ComponentScaleCandidate, Vector3>(Vector3.zero, (sum, c) => sum + c.geomAlignment * c.globalScalingDirection) / alignmentSum).normalized;
                    return new ComponentScaleCandidate(candidates.First().body, mixedLongitudinalScale, mixedOrthogonalScale, mixedAlignment, mixedScalingDirection, null);
                }
            }

            public override string ToString()
            {
                return $"{MecanimName}-{ChildMecanimName} ({segmentBody.name}-{childBody?.name})";
            }
        }



        /// <summary>
        /// Apply the type specific scaling operation to the MuJoCo component. Geoms and Joints aren't the only things that need scaling, e.g., MjInertial would also need scaling.
        /// These would need to be implemented in addition. Sharing scaling operation can be achieved by stacking cases.
        /// Current implementation only works if joints are coincident with body origins! An exception will be thrown in case any joint has non-zero local position.
        /// This is because when scaling a model our goal is to have the joint positions match the reference. With non-zero local joint positions, the parents may need to be moved as well
        /// </summary>
        internal static void Scale(this MjComponent segmentComponent, Vector3 globalParentBodyPosition, Vector3 globalScalingDirection, float longitudinalScale, float orthogonalScale, GeomScalingOptions geomOptions) 
        {
            MoveComponentWithScale(segmentComponent, globalParentBodyPosition, globalScalingDirection, longitudinalScale, orthogonalScale);
            switch (segmentComponent) 
            {
                case MjGeom geom:
                    geom.ScaleGeom(globalScalingDirection, longitudinalScale, orthogonalScale, geomOptions);
                    break;

                case MjBaseJoint:  // Already taken care of above (Condyloid joints might need adjustments, or if joint was not aligned with segment direction)
                    break;

                case MjBody:  // Already taken care of. This case is here to indicate we are happy with MjBody-s being scaled.
                    break;

                case MjSite:
                    break;

                default:
                    throw new NotImplementedException($"Scaling object {segmentComponent.name} of type {segmentComponent.GetType().Name} not implemented yet");
            }
        }


        // We could use the geom scaling built into the MuJoCo plugin, which uses the scale in the transform. It would work as geoms by definition are
        // aligned with their transforms. However, we opt for reimplementing the same scaling methods, to make the size information of the geoms
        // reliable for the user (as otherwise the scaling would need to be reapplied to those values every time they are used. Also, mesh geoms
        // would look and behave wrong.
        internal static void ScaleGeom(this MjGeom geom, Vector3 globalScalingDirection, float longitudinalScale, float orthogonalScale, GeomScalingOptions geomOptions)
        {
            var alignmentTolerance = geomOptions.alignmentTolerance;
            var attemptMisalignedScale = geomOptions.attemptMisalignedScale;
            var alignments = new List<Vector3>() { geom.transform.right, geom.transform.up, geom.transform.forward }
                                                         .Select(v => Mathf.Abs(Vector3.Dot(v, globalScalingDirection))).ToList();
            var alignedIdx = alignments.IndexOf(alignments.Max());  // ASSUMPTION 2
            switch (geom.ShapeType) 
            {
                case MjShapeComponent.ShapeTypes.Sphere:
                    geom.Sphere.Radius *= Mathf.Max(orthogonalScale, longitudinalScale);
                    break;

                case MjShapeComponent.ShapeTypes.Capsule:
                    if (alignments[1] >= 1- alignmentTolerance)
                    {
                        geom.Capsule.Radius *= orthogonalScale;
                        geom.Capsule.HalfHeight *= longitudinalScale;
                    }
                    else if (alignments[1] <= 0+ alignmentTolerance)  // Orthogonal orientation
                    {
                        longitudinalScale = 1 + (longitudinalScale - 1)*geomOptions.orthogonalCapsuleRadiusFactor;

                        var heightIncreaseFromRadiusScaling = (longitudinalScale - 1) * geom.Capsule.Radius;
                        var adjustedHeightScale = (geom.Capsule.HalfHeight * (orthogonalScale-1) - heightIncreaseFromRadiusScaling)/geom.Capsule.HalfHeight+1;

                        adjustedHeightScale = Mathf.Clamp(adjustedHeightScale, 0, Mathf.Infinity);

                        geom.Capsule.Radius *= longitudinalScale;

                        geom.Capsule.HalfHeight *= adjustedHeightScale;
                    }
                    else if(attemptMisalignedScale)
                    {
                        Debug.LogWarning($"Capsule geom {geom.name} is not aligned with either the longitudinal axis or the orthogonal plane. Attempting to scale anyway.");
                        geom.Capsule.Radius *= orthogonalScale;
                        geom.Capsule.HalfHeight *= longitudinalScale;
                    }
                    else
                    {
                        Debug.LogWarning($"Capsule geom {geom.name} is not aligned with either the longitudinal axis or the orthogonal plane. Skipping scaling this geom.");
                    }
                    break;

                case MjShapeComponent.ShapeTypes.Box:
                    var extents = new float[3] { geom.Box.Extents[0], geom.Box.Extents[1], geom.Box.Extents[2] };

                    if (alignments[alignedIdx] < 1-alignmentTolerance)
                    {
                        if(!attemptMisalignedScale)
                        {
                            Debug.LogWarning($"Box geom {geom.name} is not aligned with either the longitudinal axis or the orthogonal plane. Skipping scaling this geom.");
                            break;
                        }
                        Debug.LogWarning($"Box geom {geom.name} is not aligned with either the longitudinal axis or the orthogonal plane. Attempting to scale anyway.");
                    }

                    foreach (var idx in Enumerable.Range(0, 3).Where(i => i != alignedIdx))
                    {
                        extents[idx] *= orthogonalScale;
                    }
                    extents[alignedIdx] *= longitudinalScale;
                    geom.Box.Extents = new Vector3(extents[0], extents[1], extents[2]);
                    
                    
                    break;

                case MjShapeComponent.ShapeTypes.Ellipsoid:
                    var radiuses = new float[3] { geom.Ellipsoid.Radiuses[0], geom.Ellipsoid.Radiuses[1], geom.Ellipsoid.Radiuses[2] };


                    if (alignments[alignedIdx] < alignmentTolerance)
                    {
                        if (!attemptMisalignedScale)
                        {
                            Debug.LogWarning($"Ellipsoid geom {geom.name} is not aligned with either the longitudinal axis or the orthogonal plane. Skipping scaling this geom.");
                            break;
                        }
                        Debug.LogWarning($"Ellipsoid geom {geom.name} is not aligned with either the longitudinal axis or the orthogonal plane. Attempting to scale anyway.");
                    }

                    foreach (var idx in Enumerable.Range(0, 3).Where(i => i != alignedIdx))
                    {
                        radiuses[idx] *= orthogonalScale;
                    }
                    radiuses[alignedIdx] *= longitudinalScale;
                    geom.Ellipsoid.Radiuses = new Vector3(radiuses[0], radiuses[1], radiuses[2]);
                    

                    break;

                case MjShapeComponent.ShapeTypes.Mesh:
                    throw new NotImplementedException($"Scaling mesh geoms is not yet supported. Geom {geom.name}'s scaling is skipped.");


                default:
                    throw new NotImplementedException($"Scaling geom {geom.name} of type {geom.GetType().Name} is not yet scalable. Please implement it in Scaling.cs.");
            }
        }

        private static float GetGeomAlignment(MjBaseBody body, Vector3 dir)
        {
            List<float> results = new List<float> ();
            foreach (var geom in body.GetBodyChildComponents<MjGeom>())
            {
                var alignments = new List<Vector3>() { geom.transform.right, geom.transform.up, geom.transform.forward }
                                                             .Select(v => dir.magnitude * Mathf.Abs(Vector3.Dot(v, dir))).ToList();
                var orthogonalAlginments = alignments.Select(a => dir.magnitude * (1 - a)).ToList();
                alignments.AddRange(orthogonalAlginments);
                results.Add(alignments.Max());
            }
            return (float)results.Mean();
        }

        /// <summary>
        /// Get the first component of type T of the grandparent body (e.g. find the Hip with using the Knee).
        /// </summary>
        /// <param name="recurseUp">If true, keeps going proximal until a body with the matching component type is found. </param>
        internal static T GetPrevious<T>(this T mjComponent, bool recurseUp=true) where T: MjComponent
        {
            return MjHierarchyTool.FindParentComponent<MjBaseBody>(mjComponent).GetFirstInParentBody<T>(recurseUp);
        }

        /// <summary>
        /// Get the first component of type T more proximally than the body it was called on. 
        /// Not the same as GetComponentInParent or MjHierarchyTool.FindParentComponent, as this checks the child GameObjects of the parent MjBody
        /// </summary>
        internal static T GetFirstInParentBody<T>(this MjBaseBody body, bool recurseUp=true) where T : MjComponent
        {
            var parent = MjHierarchyTool.FindParentComponent<MjBaseBody>(body);
            if (!parent) return null;
            var parentComponent = parent.GetBodyChildComponents<T>().FirstOrDefault();  // ASSUMPTION 1
            if(!parentComponent && recurseUp) return parent.GetFirstInParentBody<T>();
            return parentComponent;
        }

        /// <summary>
        /// Iterate over all components that use this body as their MJCF parent directly (e.g. MjGeom, MjInertial, MjBaseJoint, and child MjBaseBody).
        /// </summary>
        internal static IEnumerable<T> GetBodyChildComponents<T>(this MjBaseBody body) where T : MjComponent
        {
            foreach(var childComponent in body.GetComponentsInChildren<T>())
            {
                if (MjHierarchyTool.FindParentComponent<MjBaseBody>(childComponent) == body) yield return childComponent;
            }
        }

        /// <summary>
        /// Gives the global position of where the segment corresponding to this body would start, including fixed joints (which are defined as the absence of any MjBaseJoint).
        /// This is necessary to get the scaling directions for segments that include fixed MjBodies, as they don't have joints to query positions with.
        /// </summary>
        internal static Vector3 BodySegmentStartingPosition(this MjBaseBody body)
        {
            var bodyJoint = body.GetBodyChildComponents<MjBaseJoint>().FirstOrDefault();
            return bodyJoint ? bodyJoint.transform.position : body.transform.position;
        }

        /// <summary>
        /// Get the length scaled direction of the segment section corresponding to parent body. Used in scaling segments containing multiple bodies due to fixed bodies.
        /// </summary>
        internal static Vector3 BodySegmentLengthVector(MjBaseBody parentBody, MjBaseBody childBody = null)
        {
            var childSite = parentBody.GetBodyChildComponents<MjSite>().FirstOrDefault();  // ASSUMPTION 6
            return childBody ? childBody.BodySegmentStartingPosition() - parentBody.BodySegmentStartingPosition() : childSite.transform.position - parentBody.BodySegmentStartingPosition();
        }


        /// <summary>
        /// Move a component's position as if its segment was scaled longitudinally and then orthogonally.
        /// </summary>
        internal static void MoveComponentWithScale(MjComponent component, Vector3 globalParentBodyPosition, Vector3 globalScalingDirection, float longitudinalScale, float orthogonalScale)
        {
            if (component is MjBaseJoint)
            {
                if (MjEngineTool.LocalTransformInParentBody(component).Translation != Vector3.zero)
                {
                    Debug.LogWarning($"Joint {component.name} has non-zero local position. This scaling case has not been tested.");
                }
                else
                {
                    return;
                }
            }

            var globalPositionVector = component.transform.position - globalParentBodyPosition;

            var longitudinalPositionComponent = Vector3.Dot(globalPositionVector, globalScalingDirection) * globalScalingDirection;
            var orthogonalPositionComponent = globalPositionVector - longitudinalPositionComponent;

            component.transform.position = globalParentBodyPosition + longitudinalPositionComponent * longitudinalScale + orthogonalPositionComponent * orthogonalScale;

        }

        [Serializable]
        public class GeomScalingOptions
        {
            public bool attemptMisalignedScale;
            public float alignmentTolerance;

            [Range(0f, 1f)]
            public float orthogonalCapsuleRadiusFactor;  // Capsule scaling of radius to reach a certain width can really deform the body shape

            public GeomScalingOptions(bool attemptMisalignedScale = false, float alignmentTolerance = 1e-2f, float orthogonalCapsuleRadiusFactor = 0.33f)
            {
                this.attemptMisalignedScale = attemptMisalignedScale;
                this.alignmentTolerance = alignmentTolerance;
                this.orthogonalCapsuleRadiusFactor = orthogonalCapsuleRadiusFactor;
            }

            public static GeomScalingOptions Default => new GeomScalingOptions { };
        }
    }
}
