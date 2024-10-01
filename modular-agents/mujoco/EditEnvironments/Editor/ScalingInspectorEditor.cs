using Mujoco.Extensions;
using Mujoco;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;


namespace ModularAgents.EditorScripts
{

    [CustomEditor(typeof(ScalingInspector))]
    public class ScalingInspectorEditor : Editor
    {


        [SerializeField]
        ScalingTools.ScalingSegment displaySegment;

        [SerializeField]
        bool showSegments;

        [SerializeField]
        bool showReference;

        [SerializeField]
        Vector3 referenceOffset;

        [SerializeField]
        float orthogonalScaleRatio;


       
        [SerializeField]
        bool useEndBones;


       
        [SerializeField]
        bool inheritScaleForEndEffectors;
       
        [SerializeField]
        float mass = 0;
       
        [SerializeField]
        bool repositionToReference = false;

        [SerializeField]
        bool reorientToReference = false;

        [Tooltip("The body frames will be reoriented to match the reference (it doesn't affect the actual pose).")]
        [SerializeField]
        Dictionary<(string, string), ScalingTools.MecanimBoneTransform> mecanimConnectionTransforms;


     



        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            ScalingInspector t = target as ScalingInspector;

            base.OnInspectorGUI();
            //----------------------

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

            EditorGUI.BeginDisabledGroup(!t.mjHumanoidRoot || !t.mjAvatar);
            EditorGUILayout.LabelField("Check Hierarchy is set up properly", EditorStyles.boldLabel);
            showSegments = EditorGUILayout.Toggle(new GUIContent("Show segments", "Render the scaling segments (which may span multiple bodies) " +
                                                                                 "that will be scaled to the match the lengths of corresponding " +
                                                                                 "mecanim bones in the reference. Hover over a segment name to" +
                                                                                 "display its length."), showSegments);
            if (!t.mjHumanoidRoot || !t.mjAvatar) showSegments = false;
            EditorGUI.EndDisabledGroup();
           
            EditorGUI.BeginDisabledGroup(!t.mjHumanoidRoot || !t.referenceAvatar || !t.mjAvatar);
            
            showReference = EditorGUILayout.Toggle(new GUIContent("Show reference", "Render the scaling segments (which may span multiple transforms) " +
                                                                                  "to which the MuJoCo humanoid will be scaled to match.Hover over a segment name to" +
                                                                                  "display its length."), showReference);
            if (!t.mjHumanoidRoot || !t.referenceAvatar || !t.mjAvatar) showReference = false;
            //EditorGUILayout.EndHorizontal();
            if (showReference)
            {
                referenceOffset = EditorGUILayout.Vector3Field("Reference offset", referenceOffset);
            }
            EditorGUI.EndDisabledGroup();

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

            EditorGUILayout.LabelField("Scaling Options", EditorStyles.boldLabel);

            EditorGUI.BeginDisabledGroup(!t.referenceAvatar || !t.mjHumanoidRoot || !t.mjAvatar);







            //-----------------------
            EditorGUILayout.BeginHorizontal();
            useEndBones = EditorGUILayout.Toggle(new GUIContent("Use Reference End Bones", "If enabled, end effectors like hands, toes or the head will be also scaled using available end bones in the reference."), useEndBones);
            EditorGUILayout.Space(20);
            inheritScaleForEndEffectors = EditorGUILayout.Toggle(new GUIContent("Inherit Scale", "Reuse the parent segment's scale for segment that has no corresponding length in the reference."), inheritScaleForEndEffectors);
            EditorGUILayout.EndHorizontal();

            mass = EditorGUILayout.FloatField(new GUIContent("Mass (kg)", "Leave as 0 if you want to leave the density of the geoms unaffected. Otherwise density of geoms will be set to mass/total_volume."), mass);

            orthogonalScaleRatio = EditorGUILayout.Slider(new GUIContent("Orthogonal ratio", "Ratio of longitudinal-to-orthogonal scaling. If 0, the thickness (depth and width) of segments will not be affected. At 1, a segment twice as long will be twice as wide."), orthogonalScaleRatio, 0f, 1f);

            if (!t.mjHumanoidRoot || !t.mjAvatar || ! t.referenceAvatar || !t.referenceRootGameObject) mecanimConnectionTransforms = null;


            EditorGUILayout.BeginHorizontal();
            repositionToReference = EditorGUILayout.Toggle(new GUIContent("Reposition segments", "Segments will be rotated so their child segments best overlap with the position of the reference."), repositionToReference);
            EditorGUILayout.Space(20);
            reorientToReference = EditorGUILayout.Toggle(new GUIContent("Reorient body frames", "The body frames will be reoriented to match the reference (doesn't affect the actual pose)."), reorientToReference);
            EditorGUILayout.EndHorizontal();

            if (GUILayout.Button("Scale"))
            {
                Debug.LogWarning("scaling paused");

                DoScaling(t);
            }

        }

        public void DoScaling(ScalingInspector t)
        
            {
                SetUndo(t);
                var startSegment = ScalingTools.RecursiveCreateSegments(MjHierarchyTool.FindParentComponent<MjBaseBody>(t.mjHumanoidRoot),t.mjAvatar);
                var segments = startSegment.SubtreeSegments.Skip(1).ToList();
                var mjMecanimNames = segments.Select(s => s.MecanimName).Distinct().ToList();
                var refMecanimNames = t.referenceAvatar.humanDescription.human.Select(hb => hb.humanName).ToList();

                mecanimConnectionTransforms ??= new Dictionary<(string, string), ScalingTools.MecanimBoneTransform>();

                        foreach (var segment in startSegment.SubtreeSegments.Skip(1))
                        {
                            ScalingTools.MecanimBoneTransform.TryAddNewMecanimBones(segment, t.referenceAvatar, t.referenceRootGameObject, ref mecanimConnectionTransforms);
                        }

                 List<ScalingTools.ScalingSegment.ComponentScaleCandidate> componentScaleCandidates = new List<ScalingTools.ScalingSegment.ComponentScaleCandidate>();
                    foreach (var mjMecanimName in mjMecanimNames.Where(mjName => refMecanimNames.Contains(mjName)))  // We are now guaranteed to find a length for the bone.
                    {
                        var segmentsOfMecanimBone = segments.Where(s => s.MecanimName == mjMecanimName).ToList();
                        foreach (var processedSegment in segmentsOfMecanimBone)
                        {

                            if (string.IsNullOrEmpty(processedSegment.ChildMecanimName)) continue;


                            float desiredLength = ScalingTools.CalculateDesiredSegmentLength(processedSegment.MecanimName, processedSegment.ChildMecanimName, true, mecanimConnectionTransforms);
                            float scale = desiredLength / processedSegment.SegmentLength;
                            float orthogonalScale = 1 + (scale - 1) * orthogonalScaleRatio;
                            componentScaleCandidates.AddRange(processedSegment.GetComponentScalingCandidates(scale, orthogonalScale));

                            segments.Remove(processedSegment);  // Remove processed segments so we can handle unscaled ones at the end without mecanim bones.
                        }
                    }

                        // Now we also process end effectors
                foreach (var processedSegment in segments.ToList())
                {
                    if (useEndBones && string.IsNullOrEmpty(processedSegment.ChildMecanimName))
                    {
                        float desiredLength = ScalingTools.CalculateDesiredSegmentLength(processedSegment.MecanimName, "", true, mecanimConnectionTransforms);
                        float scale = desiredLength / processedSegment.SegmentLength;
                        componentScaleCandidates.AddRange(processedSegment.GetComponentScalingCandidates(scale));
                        segments.Remove(processedSegment);
                    }
                    else if (inheritScaleForEndEffectors && string.IsNullOrEmpty(processedSegment.ChildMecanimName))
                    {
                        Vector2 scales = processedSegment.ParentSegment.AppliedScale;
                        componentScaleCandidates.AddRange(processedSegment.GetComponentScalingCandidates(scales[0], scales[1]));
                        segments.Remove(processedSegment);
                    }
                }
                if (segments.Count > 0) Debug.Log($"The following segments were not scaled: {string.Join(", ", segments.Select(seg => seg.ToString()))}");


                var componentsToScale = componentScaleCandidates.GroupBy(cand => cand.body)
                                                                .Select(grp => ScalingTools.ScalingSegment.ComponentScaleCandidate.WeightedMix(grp));
                foreach (var component in componentsToScale)
                {
                    component.Scale();
                }

                if (mass != 0)
                {
                    var geoms = t.mjHumanoidRoot.GetComponentInParent<MjBaseBody>().GetComponentsInChildren<MjGeom>();
                    var totalMass = geoms.Sum(g => g.Mass != 0 ? g.Mass : g.GetVolume() * g.Density);
                    var massScale = mass / totalMass;
                    foreach (var geom in geoms)
                    {
                        if (geom.Mass != 0) geom.Mass *= massScale;
                        else geom.Density *= massScale;
                    }
                }

                //Segments are the correct length, but not the correct orientation. We can now adjust for that.
                if (repositionToReference)
                {
                    var originalJointOrientations = new Dictionary<int, Quaternion>(
                        t.mjHumanoidRoot
                            .GetComponentInParent<MjBaseBody>()
                            .GetComponentsInChildren<MjBaseJoint>()
                            .Select(j => new KeyValuePair<int, Quaternion>(j.GetInstanceID(), j.transform.rotation))
                            .ToList()
                    );

                    foreach (var processedSegment in startSegment.SubtreeSegments.Skip(1).ToList())
                    {

                        if (string.IsNullOrEmpty(processedSegment.ChildMecanimName)) continue;

                        var refBoneTransform = mecanimConnectionTransforms[(processedSegment.MecanimName, processedSegment.ChildMecanimName)];

                        Reposition.AlignBodyPosition(processedSegment.childBody,
                            refBoneTransform.childGlobalPosition + t.mjHumanoidRoot.transform.position,
                            processedSegment.ParentSegment.ChildSegments.Count < 2,
                            originalJointOrientations);
                    }
                }

                // Change the body frame axis to match the reference.
                if (reorientToReference)
                {
                    foreach (var processedSegment in startSegment.SubtreeSegments.Skip(1).ToList())
                    {

                        if (string.IsNullOrEmpty(processedSegment.MecanimName)) continue;
                        if (string.IsNullOrEmpty(processedSegment.ChildMecanimName)) continue;

                        var refBoneTransform = mecanimConnectionTransforms[(processedSegment.MecanimName, processedSegment.ChildMecanimName)];

                        Reposition.ReorientBodyFrame(processedSegment.segmentBody, refBoneTransform.GlobalRotation);
                    }
                 }

            // Ensure the transform scale of Mj components is 1:
            foreach (MjComponent childComponent in t.mjHumanoidRoot.transform.parent.GetComponentsInChildren<MjComponent>())
            {
                childComponent.transform.localScale = Vector3.one;
            }

        }
        
        void OnSceneGUI()
        {
            
            ScalingInspector t = target as ScalingInspector;

            if ((showSegments || showReference) && t.mjHumanoidRoot)
            {


                var gradient = new Gradient();
                var colors = new GradientColorKey[5];
                var alphas = new GradientAlphaKey[2];

                colors[0] = new GradientColorKey(Color.red, 0);
                colors[1] = new GradientColorKey(Color.blue, 0.25f);
                colors[2] = new GradientColorKey(Color.green, 0.5f);
                colors[3] = new GradientColorKey(Color.cyan, 0.75f);
                colors[4] = new GradientColorKey(Color.yellow, 1f);
                alphas[0] = new GradientAlphaKey(1, 0);
                alphas[1] = new GradientAlphaKey(1, 1);

                gradient.SetKeys(colors, alphas);

                var startSegment = ScalingTools.RecursiveCreateSegments(MjHierarchyTool.FindParentComponent<MjBaseBody>(t.mjHumanoidRoot), t.mjAvatar);


                var maxTreeElements = startSegment.SubtreeSegments.Skip(1).Count();
                var maxSubTreeElements = startSegment.SubtreeSegments.Skip(1).Select(s => s.SubtreeSegments.Count()).Max();

                if (showSegments)
                {
                    foreach (var segment in startSegment.SubtreeSegments.Skip(1))
                    {
                        Handles.color = gradient.Evaluate((float)segment.SubtreeSegments.Count() / maxSubTreeElements);

                        Handles.DrawLine(segment.StartPoint, segment.EndPoint, 4f);

                        GUI.color = Handles.color;
                        Handles.Label((segment.StartPoint * 0.6f + segment.EndPoint * 0.4f),
                            new GUIContent(segment.MecanimName, $"{segment} length: {segment.SegmentLength}"));

                        Handles.color = new Color(Handles.color[0], Handles.color[1], Handles.color[2], 0.5f);
                        Handles.SphereHandleCap(0, segment.EndPoint, Quaternion.identity, 0.02f, EventType.Repaint);

                    }
                }
                if (showReference)
                {

                    mecanimConnectionTransforms ??= new Dictionary<(string, string), ScalingTools.MecanimBoneTransform>();

                    foreach (var segment in startSegment.SubtreeSegments.Skip(1))
                    {
                        ScalingTools.MecanimBoneTransform.TryAddNewMecanimBones(segment, t.referenceAvatar, t.referenceRootGameObject, ref mecanimConnectionTransforms);
                    }

                    Vector3 offset = referenceOffset + t.mjHumanoidRoot.transform.position;

                    foreach (var mecanimBoneTransform in mecanimConnectionTransforms.Values)
                    {
                        Handles.color = gradient.Evaluate((float)mecanimBoneTransform.subtreeSize / maxSubTreeElements);
                        GUI.color = Handles.color;
                        if (string.IsNullOrEmpty(mecanimBoneTransform.childMecanimName) && mecanimBoneTransform.childGlobalPosition == Vector3.zero)
                        {
                            Handles.color = new Color(Handles.color[0], Handles.color[1], Handles.color[2], 0.5f);
                            Handles.SphereHandleCap(0, mecanimBoneTransform.GlobalPosition + offset, Quaternion.identity, 0.04f, EventType.Repaint);
                            Handles.Label(mecanimBoneTransform.GlobalPosition + offset, mecanimBoneTransform.mecanimName);
                            continue;
                        }

                        Handles.DrawLine(mecanimBoneTransform.GlobalPosition + offset, mecanimBoneTransform.childGlobalPosition + offset, 4f);

                        var desiredLength = ScalingTools.CalculateDesiredSegmentLength(mecanimBoneTransform.mecanimName, $"{mecanimBoneTransform.childMecanimName}", true, mecanimConnectionTransforms);
                        Handles.Label((mecanimBoneTransform.GlobalPosition * 0.6f + mecanimBoneTransform.childGlobalPosition * 0.4f) + offset,
                            new GUIContent(mecanimBoneTransform.mecanimName, $"{mecanimBoneTransform} length: {desiredLength}"));

                        Handles.color = new Color(Handles.color[0], Handles.color[1], Handles.color[2], 0.5f);
                        Handles.SphereHandleCap(0, mecanimBoneTransform.childGlobalPosition + offset, Quaternion.identity, 0.02f, EventType.Repaint);
                    }

                }
                else
                {
                    mecanimConnectionTransforms = null;
                }

            }
        }
      

        private void SetUndo(ScalingInspector t)
        {


            Undo.RegisterCompleteObjectUndo(t.mjHumanoidRoot.transform.parent.GetComponentsInChildren<Component>().ToArray(), "Scale Mj Humanoid Components");
            Undo.RegisterCompleteObjectUndo(t.mjHumanoidRoot.transform.parent.GetComponentsInChildren<Transform>().ToArray(), "Scale Mj Humanoid Transforms");
        }
      

    }




}



