using JetBrains.Annotations;
using MathNet.Numerics.Statistics;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;


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
    public static class ScalingTools

    {

        /// <summary>
        /// Find the corresponding mecanim bones in the Unity avatar, and get that bone's length. If symmetric arguement is enabled, will return the average bone length for bilateral bones.
        /// </summary>
        public static float CalculateDesiredSegmentLength(string startMecanimName, string endMecanimName, bool symmetric, Dictionary<(string, string), MecanimBoneTransform> positionDict)
        {

            var processedEndMecanimName = symmetric ? SideAgnostic(endMecanimName) : endMecanimName;
            var processedStartMecanimName = symmetric ? SideAgnostic(startMecanimName) : startMecanimName;

            bool handlingSymmetricEnd = processedEndMecanimName != endMecanimName;
            bool handlingSymmetricStart = processedStartMecanimName != startMecanimName;
            if (handlingSymmetricEnd && !handlingSymmetricStart)
            {
                return (CalculateDesiredSegmentLength(startMecanimName, "Left" + processedEndMecanimName, false, positionDict) +
                        CalculateDesiredSegmentLength(startMecanimName, "Right" + processedEndMecanimName, false, positionDict)) / 2;
            }
            else if (handlingSymmetricEnd && handlingSymmetricStart)
            {
                return (CalculateDesiredSegmentLength("Left" + processedStartMecanimName, "Left" + processedEndMecanimName, false, positionDict) +
                    CalculateDesiredSegmentLength("Right" + processedStartMecanimName, "Right" + processedEndMecanimName, false, positionDict)) / 2;
            }
            else if (!handlingSymmetricEnd && handlingSymmetricStart)
            {
                return (CalculateDesiredSegmentLength("Left" + processedStartMecanimName, processedEndMecanimName, false, positionDict) +
                    CalculateDesiredSegmentLength("Right" + processedStartMecanimName, processedEndMecanimName, false, positionDict)) / 2;
            }

            var refConnection = positionDict[(processedStartMecanimName, processedEndMecanimName)];

            return (refConnection.childGlobalPosition - refConnection.GlobalPosition).magnitude;
        }


        public static string BodyToMecanimName(MjBaseBody body, Avatar mjAvatar)
        {

            return mjAvatar.humanDescription.human.FirstOrDefault(hb => hb.boneName == body.name).humanName;
        }


        /// <summary>
        /// Segment creation is performed proximal->distal (scaling will be distal->proximal). Must start with a body corresponding to a MecanimBone.
        /// </summary>
        public static ScalingSegment RecursiveCreateSegments(MjBaseBody curBody, Avatar mjAvatar, ScalingSegment parentSegment = null)
        {
            var mecanimName = BodyToMecanimName(curBody, mjAvatar);

            var startBody = curBody;
            var childMecanimBodies = GetChildMecanimBodies(curBody, mjAvatar).ToList();

            parentSegment ??= new ScalingSegment(curBody, curBody);

            if (childMecanimBodies.Count == 0) // We are at an end effector
            {
                foreach (var eeSite in GetEndEffectorSites(curBody))
                {
                    ScalingSegment segment = new ScalingSegment(startBody, eeSite);
                    segment.ParentSegment = parentSegment;
                    segment.MecanimName = mecanimName;
                    parentSegment.ChildMecanimName = mecanimName;
                }
            }

            foreach (var childMecanimBody in childMecanimBodies)
            {
                ScalingSegment segment = new ScalingSegment(startBody, childMecanimBody);
                segment.ParentSegment = parentSegment;
                segment.MecanimName = mecanimName;
                parentSegment.ChildMecanimName = mecanimName;

                //Debug.Log($"Segment: {mecanimName}, Body: {segment.segmentBody.name}, Bodies: {string.Join(", ", segment.segmentBodies.Select(bd => bd.name))}, Child body: {segment.childBody.name}");
                RecursiveCreateSegments(childMecanimBody, mjAvatar, segment);
            }

            return parentSegment;

        }


        /// <summary>
        /// The first child bodies depthwise along all branches of the kinematic tree that are also Mecanim bones.
        /// </summary>
        public static IEnumerable<MjBaseBody> GetChildMecanimBodies(MjBaseBody body, Avatar mjAvatar)
        {
            foreach (var childBody in body.GetBodyChildComponents<MjBaseBody>())
            {
                var childMecanimName = BodyToMecanimName(childBody, mjAvatar);
                if (!string.IsNullOrEmpty(childMecanimName)) yield return childBody;
                else
                {
                    foreach (var recursiveChildBody in GetChildMecanimBodies(childBody, mjAvatar))
                    {
                        yield return recursiveChildBody;
                    }
                }
            }
        }

        public static IEnumerable<MjSite> GetEndEffectorSites(MjBaseBody body)
        {
            if (body.GetComponentInChildren<MjBody>())
            {
                foreach (var child in body.GetBodyChildComponents<MjBaseBody>())
                {
                    GetEndEffectorSites(child);
                }
            }
            if (body.GetComponentsInChildren<MjSite>().Count() > 1) Debug.LogError($"More than one site in end effector {body.name}, please implement how to handle this case.");
            var site = body.GetComponentInChildren<MjSite>();
            if (!site) Debug.LogError($"No end effector site included in body {body.name}!");
            yield return site;
        }

        public static string SideAgnostic(string mecanimName) => mecanimName.Replace("Left", "").Replace("Right", "");


        /// <summary>
        /// Gives positional information about mecanim bones in an avatar with a given transform hierarchy. 
        /// Using the hierarchy (from referenceRootGameObject) directly would ignore the adjustments made to the avatar by the humanoid rigging.
        /// This class uses the structure found in the transform hierarchy of referenceRootGameobject, with the position and rotation cross-referenced from
        /// the flattened list of the avatar skeleton bone collection. We could get the length only from the hierarchy, but we might as well do this
        /// for the visualization of the avatar since we have all the information for it.
        /// </summary>
        public class MecanimBoneTransform
        {
            public Matrix4x4 transform;

            public Vector3 GlobalPosition => transform.GetPosition();
            public Quaternion GlobalRotation => transform.rotation;

            public string mecanimName;
            public string childMecanimName;

            public Vector3 childGlobalPosition;

            public int subtreeSize;

            private MecanimBoneTransform(Matrix4x4 transform, ScalingSegment segment)
            {
                this.transform = transform;
                mecanimName = segment.MecanimName;
                childMecanimName = segment.ChildMecanimName;
            }

            public static void TryAddNewMecanimBones(ScalingSegment segment, Avatar referenceAvatar, GameObject referenceRootGameObject, ref Dictionary<(string, string), MecanimBoneTransform> existingBones)
            {
                if (existingBones.ContainsKey((segment.MecanimName, $"{segment.ChildMecanimName}")))  // String interpolation, as ChildMecanimName may be null
                {
                    return;
                }

                var boneName = referenceAvatar.humanDescription.human.FirstOrDefault(hb => hb.humanName == segment.MecanimName).boneName;
                var bone = referenceAvatar.humanDescription.skeleton.FirstOrDefault(sb => sb.name == boneName);

                MecanimBoneTransform currentBoneTransform;

                if (segment.MecanimName == "Hips")
                {
                    var preTransform = AggregateTransformBetweenMecanimBones("", segment.MecanimName, referenceAvatar, referenceRootGameObject);
                    var transform = preTransform * Matrix4x4.TRS(bone.position, bone.rotation, bone.scale);

                    currentBoneTransform = new MecanimBoneTransform(Matrix4x4.TRS(Vector3.zero, transform.rotation, transform.lossyScale), segment);
                    existingBones.Add((currentBoneTransform.mecanimName, currentBoneTransform.childMecanimName), currentBoneTransform);
                }
                else
                {

                    var parentMecanimName = segment.ParentSegment.MecanimName;

                    Debug.Assert(existingBones.ContainsKey((parentMecanimName, segment.MecanimName)));

                    var parentTransform = existingBones[(parentMecanimName, segment.MecanimName)].transform;
                    var transform = parentTransform * AggregateTransformBetweenMecanimBones(segment.ParentSegment.MecanimName, segment.MecanimName, referenceAvatar, referenceRootGameObject);

                    currentBoneTransform = new MecanimBoneTransform(transform, segment);

                    existingBones.Add((currentBoneTransform.mecanimName, $"{currentBoneTransform.childMecanimName}"), currentBoneTransform);  // String interpolation used to handle null cases
                }

                if (!string.IsNullOrEmpty(currentBoneTransform.childMecanimName))
                {
                    currentBoneTransform.childGlobalPosition = (currentBoneTransform.transform * AggregateTransformBetweenMecanimBones(currentBoneTransform.mecanimName, currentBoneTransform.childMecanimName, referenceAvatar, referenceRootGameObject)).GetPosition();
                }
                else
                {
                    var endBoneTransform = GetReferenceEndBoneAvatarTransform(currentBoneTransform.mecanimName, referenceAvatar, referenceRootGameObject);
                    if (endBoneTransform != Matrix4x4.identity) currentBoneTransform.childGlobalPosition = (currentBoneTransform.transform * endBoneTransform).GetPosition();
                }

                currentBoneTransform.subtreeSize = segment.SubtreeSegments.Count();
            }

            private static Matrix4x4 AggregateTransformBetweenMecanimBones(string startMecanimName, string endMecanimName, Avatar referenceAvatar, GameObject referenceRootGameObject)
            {
                var startBoneName = GetSkeletonName(startMecanimName, referenceAvatar);
                var endTransform = GetReferenceMecanimTransform(endMecanimName, referenceAvatar, referenceRootGameObject);
                var parents = endTransform.GetComponentsInParent<Transform>(true).ToList();
                List<string> subChainSkeletonBoneNames = parents.Select(t => t.name)
                                                                .Reverse()
                                                                .ToList();

                List<SkeletonBone> subSkeleton = subChainSkeletonBoneNames.Select(n => referenceAvatar.humanDescription.skeleton.First(sb => sb.name.Replace("(Clone)", "") == n)).ToList();
                if (!string.IsNullOrEmpty(startBoneName))
                {
                    subSkeleton = subSkeleton.SkipWhile(sb => sb.name != startBoneName).Skip(1).ToList();
                }

                return subSkeleton.Aggregate(Matrix4x4.identity, (runningTotal, curBone) => runningTotal * Matrix4x4.TRS(curBone.position, curBone.rotation, curBone.scale));
            }

            public override string ToString()
            {
                return $"{mecanimName}-{childMecanimName}";
            }

        }


        /// <summary>
        /// Get the SkeletonBone name of the transform in the avatar corresponding to the given HumanBone name.
        /// </summary>
        public static string GetSkeletonName(string humanName, Avatar referenceAvatar)
        {
            return referenceAvatar.humanDescription.human.FirstOrDefault(hb => hb.humanName == humanName).boneName;
        }


        /// <summary>
        /// Get the transform that corresponds to a HumanBone name in the hierarchy of the avatar (the referenceRootGameobject contains the hierarchical relationship, the avatar the mapping from human to skeleton)
        /// </summary>
        public static Transform GetReferenceMecanimTransform(string humanoidBoneName, Avatar referenceAvatar, GameObject referenceRootGameObject)
        {
            var boneName = GetSkeletonName(humanoidBoneName, referenceAvatar);
            var transformFound = referenceRootGameObject.GetComponentsInChildren<Transform>().First(t => t.name == boneName);
            return transformFound;
        }

        /// <summary>
        /// Checks if there is an end bone for the bone, and returns the local transform matrix from which, e.g, we can attempt to get length of end effectors.
        /// We could parametrize the end bone naming convention.
        /// </summary>
        public static Matrix4x4 GetReferenceEndBoneAvatarTransform(string humanoidBoneName, Avatar referenceAvatar, GameObject referenceRootGameObject)
        {
            var boneName = GetSkeletonName(humanoidBoneName, referenceAvatar);
            var transformFound = referenceRootGameObject.GetComponentsInChildren<Transform>().FirstOrDefault(t => t.name.ToLower().Contains(boneName.ToLower()) && t.name.ToLower().Contains("end"));

            if (!transformFound) return Matrix4x4.identity;


            var endBoneName = transformFound.name;
            var endBone = referenceAvatar.humanDescription.skeleton.FirstOrDefault(sb => sb.name == endBoneName);

            return Matrix4x4.TRS(endBone.position, endBone.rotation, endBone.scale);
        }




        /// <summary>
        /// We define a ScalingSegment as the spatial connection between a joint that couples two bodies, and the joint(s) of the proximal body.
        /// A ScalingSegment corresponds to a body segment (e.g. thigh, shank), with a cylindrical representation aligned with the two joints.
        /// For this reason we assume that if there are multiple joints in a body, then they are coincident.
        /// This is closer to the way animation rigs are structured, and helps getting the correct axes and magnitude of scaling necessary.
        /// </summary>
        public class ScalingSegment
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
        public static IEnumerable<T> GetBodyChildComponents<T>(this MjBaseBody body) where T : MjComponent
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
