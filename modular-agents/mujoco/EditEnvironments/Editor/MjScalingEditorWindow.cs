using UnityEditor;
using UnityEngine;
using System.Linq;
using MathNet.Numerics.Statistics;
using System.Collections.Generic;
using static UnityEditor.PlayerSettings;
using System.Drawing;
using Color = UnityEngine.Color;
using System;

namespace Mujoco.Extensions
{
    public class MjScalingEditorWindow : EditorWindow
    {
        [SerializeField]
        MjFreeJoint mjHumanoidRoot;

        [SerializeField]
        Avatar referenceAvatar;

        [SerializeField]
        GameObject referenceRootGameObject;

        [SerializeField]
        Avatar mjAvatar;


        [SerializeField]
        ScalingTools.ScalingSegment displaySegment;

        [SerializeField]
        bool showSegments;

        [SerializeField]
        bool showReference;

        [SerializeField]
        bool reorientToReference;

        [SerializeField]
        bool repositionToReference;

        [SerializeField]
        bool inheritScaleForEndEffectors;

        [SerializeField]
        bool useEndBones;

        [SerializeField]
        Vector3 referenceOffset;

        [SerializeField]
        Dictionary<(string, string), ScalingTools.MecanimBoneTransform> mecanimConnectionTransforms;

        [SerializeField]
        float mass;

        [SerializeField]
        float orthogonalScaleRatio;



        [MenuItem("Tools/Scale MuJoCo Humanoid")]
        public static void ShowScalingEditor()
        {
            // This method is called when the user selects the menu item in the Editor
            EditorWindow wnd = GetWindow<MjScalingEditorWindow>();
            wnd.titleContent = new GUIContent("Scale MuJoCo Humanoid");

            // Limit size of the window
            wnd.minSize = new Vector2(500, 200);
            wnd.maxSize = new Vector2(1920, 720);

            wnd.Show();
        }

        void OnEnable()
        {
            SceneView.duringSceneGui += this.OnSceneGUI;
        }

        void OnDisable()
        {
            SceneView.duringSceneGui -= this.OnSceneGUI;
        }

        void OnSceneGUI(SceneView sceneView)
        {
            if ((showSegments || showReference) && mjHumanoidRoot)
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

                var startSegment = ScalingTools.RecursiveCreateSegments(MjHierarchyTool.FindParentComponent<MjBaseBody>(mjHumanoidRoot), mjAvatar);


                var maxTreeElements = startSegment.SubtreeSegments.Skip(1).Count();
                var maxSubTreeElements = startSegment.SubtreeSegments.Skip(1).Select(s => s.SubtreeSegments.Count()).Max();

                if (showSegments)
                {
                    foreach (var segment in startSegment.SubtreeSegments.Skip(1))
                    {
                        Handles.color = gradient.Evaluate((float)segment.SubtreeSegments.Count() / maxSubTreeElements);

                        Handles.DrawLine(segment.StartPoint, segment.EndPoint, 4f);

                        GUI.color = Handles.color;
                        Handles.Label((segment.StartPoint* 0.6f + segment.EndPoint *0.4f), 
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
                        ScalingTools.MecanimBoneTransform.TryAddNewMecanimBones(segment, referenceAvatar, referenceRootGameObject, ref mecanimConnectionTransforms);
                    }

                    Vector3 offset = referenceOffset + mjHumanoidRoot.transform.position;

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
                        Handles.Label((mecanimBoneTransform.GlobalPosition * 0.6f + mecanimBoneTransform.childGlobalPosition * 0.4f)  + offset, 
                            new GUIContent(mecanimBoneTransform.mecanimName, $"{mecanimBoneTransform} length: {desiredLength}") );

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

        public void OnGUI()
        {
            EditorStyles.label.wordWrap = true;
            EditorGUILayout.BeginVertical();

            EditorGUILayout.LabelField("Assumptions:\r\n" +
                "1. All joints of a body are coincident.\r\n" +
                "2. Capsules, boxes, cylinders are all aligned with the longitudinal segment axis, or with the orthogonal plane.\r\n" +
                "3. Only longitudinally and uniformly in orthogonal directions are scaled (anatomical pose unchanged by scaling).\r\n" +
                "4. Joints are coincident with body frame.\r\n" +
                "5. End effectors have a site marking the end of their segment.\r\n" +
                "6. Assume that branching bodies are only in the torso and aligned with the global vertical direction.\r\n" +
                "\r\nHover over the field labels for tooltip information.");

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

            var avatarObject = EditorGUILayout.ObjectField(new GUIContent("Reference Avatar", "The Mecanim avatar of the Unity humanoid to which the MuJoCo humanoid will be scaled."),
                referenceAvatar, typeof(Avatar), true);
            referenceAvatar = avatarObject as Avatar;
            var hierarchyObject = EditorGUILayout.ObjectField(new GUIContent("Reference Root", "The root of the transform hierarchy in the same FBX file of the reference avatar."), 
                referenceRootGameObject, typeof(GameObject), true);
            referenceRootGameObject = hierarchyObject as GameObject;

            EditorGUILayout.Space(20);

            var mjAvatarObject = EditorGUILayout.ObjectField(new GUIContent("MuJoCo Avatar", "Mapping of the MujoCo humanoid segments to the Mecanim format. Used to find matches between the reference and MuJoCo segments. Use the FBX exporter package to create this."), 
                mjAvatar, typeof(Avatar), true);
            mjAvatar = mjAvatarObject as Avatar;
            var humanoidObject = EditorGUILayout.ObjectField(new GUIContent("MuJoCo Humanoid", "The MuJoCo humanoid, the segments of which will be scaled to match the reference avatar."), 
                mjHumanoidRoot, typeof(MjFreeJoint), true);
            mjHumanoidRoot = humanoidObject as MjFreeJoint;;

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

            EditorGUILayout.BeginVertical();

            EditorGUILayout.BeginHorizontal();
            EditorGUI.BeginDisabledGroup(!mjHumanoidRoot || !mjAvatar);
            showSegments = EditorGUILayout.Toggle(new GUIContent("Show segments", "Render the scaling segments (which may span multiple bodies) " +
                                                                                  "that will be scaled to the match the lengths of corresponding " +
                                                                                  "mecanim bones in the reference. Hover over a segment name to" +
                                                                                  "display its length."), showSegments);
            if (!mjHumanoidRoot || !mjAvatar) showSegments = false;
            EditorGUI.EndDisabledGroup();
            EditorGUILayout.Space(20);
            EditorGUI.BeginDisabledGroup(!mjHumanoidRoot || !referenceAvatar || !mjAvatar);
            showReference = EditorGUILayout.Toggle(new GUIContent("Show reference", "Render the scaling segments (which may span multiple transforms) " +
                                                                                  "to which the MuJoCo humanoid will be scaled to match.Hover over a segment name to" +
                                                                                  "display its length."), showReference);
            if (!mjHumanoidRoot || !referenceAvatar || !mjAvatar) showReference = false;
            EditorGUILayout.EndHorizontal();
            if (showReference)
            {
                referenceOffset = EditorGUILayout.Vector3Field("Reference offset", referenceOffset);
            }
            EditorGUI.EndDisabledGroup();

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

            EditorGUI.BeginDisabledGroup(!referenceAvatar || !mjHumanoidRoot || !mjAvatar);

            EditorGUILayout.BeginHorizontal();
            useEndBones = EditorGUILayout.Toggle(new GUIContent("Use Reference End Bones", "If enabled, end effectors like hands, toes or the head will be also scaled using available end bones in the reference."), useEndBones);
            EditorGUILayout.Space(20);
            inheritScaleForEndEffectors = EditorGUILayout.Toggle(new GUIContent("Inherit Scale", "Reuse the parent segment's scale for segment that has no corresponding length in the reference."), inheritScaleForEndEffectors);
            EditorGUILayout.EndHorizontal();

            mass = EditorGUILayout.FloatField(new GUIContent("Mass (kg)", "Leave as 0 if you want to leave the density of the geoms unaffected. Otherwise density of geoms will be set to mass/total_volume."), mass);

            orthogonalScaleRatio = EditorGUILayout.Slider(new GUIContent("Orthogonal ratio", "Ratio of longitudinal-to-orthogonal scaling. If 0, the thickness (depth and width) of segments will not be affected. At 1, a segment twice as long will be twice as wide."), orthogonalScaleRatio, 0f, 1f);

            if (!mjHumanoidRoot || !mjAvatar || !referenceAvatar || !referenceRootGameObject) mecanimConnectionTransforms = null;

            EditorGUILayout.BeginHorizontal();
            repositionToReference = EditorGUILayout.Toggle(new GUIContent("Reposition segments", "Segments will be rotated so their child segments best overlap with the position of the reference."), repositionToReference);
            EditorGUILayout.Space(20);
            reorientToReference = EditorGUILayout.Toggle(new GUIContent("Reorient body frames", "The body frames will be reoriented to match the reference (doesn't affect the actual pose)."), reorientToReference);
            EditorGUILayout.EndHorizontal();

            if (GUILayout.Button("Scale Humanoid"))
            {
                SetUndo();
                var startSegment = ScalingTools.RecursiveCreateSegments(MjHierarchyTool.FindParentComponent<MjBaseBody>(mjHumanoidRoot), mjAvatar   );
                var segments = startSegment.SubtreeSegments.Skip(1).ToList();
                var mjMecanimNames = segments.Select(s => s.MecanimName).Distinct().ToList();
                var refMecanimNames = referenceAvatar.humanDescription.human.Select(hb => hb.humanName).ToList();

                mecanimConnectionTransforms ??= new Dictionary<(string, string), ScalingTools.MecanimBoneTransform>();

                foreach (var segment in startSegment.SubtreeSegments.Skip(1))
                {
                    ScalingTools.MecanimBoneTransform.TryAddNewMecanimBones(segment, referenceAvatar, referenceRootGameObject, ref mecanimConnectionTransforms);
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
                    if(useEndBones && string.IsNullOrEmpty(processedSegment.ChildMecanimName))
                    {
                        float desiredLength = ScalingTools.CalculateDesiredSegmentLength(processedSegment.MecanimName, "", true, mecanimConnectionTransforms);
                        float scale = desiredLength / processedSegment.SegmentLength;
                        componentScaleCandidates.AddRange(processedSegment.GetComponentScalingCandidates(scale));
                        segments.Remove(processedSegment);
                    }
                    else if(inheritScaleForEndEffectors && string.IsNullOrEmpty (processedSegment.ChildMecanimName))
                    {
                        Vector2 scales = processedSegment.ParentSegment.AppliedScale;
                        componentScaleCandidates.AddRange(processedSegment.GetComponentScalingCandidates(scales[0], scales[1]));
                        segments.Remove(processedSegment);
                    }
                }
                if(segments.Count > 0) Debug.Log($"The following segments were not scaled: {string.Join(", ", segments.Select(seg => seg.ToString()))}");


                var componentsToScale = componentScaleCandidates.GroupBy(cand => cand.body)
                                                                .Select(grp => ScalingTools.ScalingSegment.ComponentScaleCandidate.WeightedMix(grp));
                foreach (var component in componentsToScale) 
                {
                    component.Scale();                
                }

                if(mass != 0)
                {
                    var geoms = mjHumanoidRoot.GetComponentInParent<MjBaseBody>().GetComponentsInChildren<MjGeom>();
                    var totalMass = geoms.Sum(g =>g.Mass!=0? g.Mass :  g.GetVolume()*g.Density);
                    var massScale = mass/totalMass;
                    foreach(var geom in geoms) 
                    {
                        if(geom.Mass != 0) geom.Mass *= massScale;
                        else geom.Density *= massScale;
                    }
                }

                //Segments are the correct length, but not the correct orientation. We can now adjust for that.
                if (repositionToReference)
                {
                    var originalJointOrientations = new Dictionary<int, Quaternion>(
                        mjHumanoidRoot
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
                            refBoneTransform.childGlobalPosition + mjHumanoidRoot.transform.position, 
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
                foreach (MjComponent childComponent in mjHumanoidRoot.transform.parent.GetComponentsInChildren<MjComponent>())
                {
                    childComponent.transform.localScale = Vector3.one;
                }

            }
            EditorGUI.EndDisabledGroup();

           

            EditorGUILayout.EndVertical();

            EditorGUILayout.EndVertical();
            SceneView.RepaintAll();
        }

        private void SetUndo()
        {
            Undo.RegisterCompleteObjectUndo(mjHumanoidRoot.transform.parent.GetComponentsInChildren<Component>().ToArray(), "Scale Mj Humanoid Components");
            Undo.RegisterCompleteObjectUndo(mjHumanoidRoot.transform.parent.GetComponentsInChildren<Transform>().ToArray(), "Scale Mj Humanoid Transforms");
        }

      
        private IEnumerable<MjSite> GetEndEffectorSites(MjBaseBody body)
        {
            if (body.GetComponentInChildren<MjBody>())
            {
                foreach(var child in body.GetBodyChildComponents<MjBaseBody>())
                {
                    GetEndEffectorSites(child);
                }
            }
            if (body.GetComponentsInChildren<MjSite>().Count() > 1) Debug.LogError($"More than one site in end effector {body.name}, please implement how to handle this case.");
            var site = body.GetComponentInChildren<MjSite>();
            if (!site) Debug.LogError($"No end effector site included in body {body.name}!");
            yield return site;
        }

        private string SideAgnostic(string mecanimName) => mecanimName.Replace("Left", "").Replace("Right", "");

      

    }


}