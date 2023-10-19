using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using HDF.PInvoke;
using System.Runtime.InteropServices;
using System;
using Unity.VisualScripting;
using UnityEditor;
using System.Linq;
using Mujoco;
using Mujoco.Extensions;
using MathNet.Numerics.Interpolation;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics;
using ModularAgents;

public class Hdf5Loader : MonoBehaviour
{
    [SerializeField]
    string filePath;

    List<string> prefixes;

    [SerializeField]
    TextAsset subsetJson;

    [SerializeField]
    List<string> fields = new List<string> { "position", "quaternion", "joints", "velocity", "angular_velocity", "joints_velocity", "body_positions", "body_quaternions" };

    [SerializeField]
    MjFreeJoint root;

    public MjFreeJoint Root => root;

    public int CurClipIdx { get; set; }

    public int CurFrameIndexInClip { get; set; }
    public float TimeInClip { get => CurFrameIndexInClip * Time.fixedDeltaTime; set => CurFrameIndexInClip = (int)(value/Time.fixedDeltaTime); }
    public float NormalizedTimeInClip { get => (float)CurFrameIndexInClip / motionFiles[CurClipIdx].Length; set => CurFrameIndexInClip = (int)(value * motionFiles[CurClipIdx].Length); }

    public float[] CurQpos => motionFiles[CurClipIdx].Qpos(CurFrameIndexInClip);
    public float[] CurQvel => motionFiles[CurClipIdx].Qvel(CurFrameIndexInClip);

    [SerializeField]
    bool loopClips;

    public event EventHandler<MocapPlaybackEventArgs> ClipEndedEvent;
    public event EventHandler<MocapPlaybackEventArgs> ClipsRanOutEvent;

    public IReadOnlyList<CmuMotionData> motionFiles;

    // Start is called before the first frame update
    private void Awake()
    {
        var subset = JsonUtility.FromJson<DataSubsetNameCollection>(subsetJson.text);

        prefixes = subset.names.Select(n => $"{n}/walkers/walker_0/").ToList();
        
        var motionFiles = new List<CmuMotionData>();

        long fileId = H5F.open(filePath, H5F.ACC_RDONLY);
        foreach(var prefix in prefixes)
        {
            motionFiles.Add(new CmuMotionData(fileId, prefix, fields, mocapDt:subset.mocap_dt));
        }
        H5F.close(fileId);

        this.motionFiles = motionFiles;
        foreach(var mf in this.motionFiles)
        {
            Debug.Log(mf.prefix + ": " + string.Join(", ", mf.Keys));
            Debug.Log(mf.prefix + ": " + mf.Keys.Count() +"; "+ mf[fields[0]].Length +", "+ mf[fields[0]][0].Length);
        }
        //MjState.ExecuteAfterMjStart(SetMjState);
        //MjScene.Instance.preUpdateEvent += (_, _) => SetMjState();

        
    }

    

    public unsafe void SetMjState(int clipIdx, int frameIdx)
    {
        //IncrementFrame();

        var frame = motionFiles[clipIdx].Qpos(frameIdx);
        for (int i = 0; i < frame.Length; i++)
        {
            MjScene.Instance.Data->qpos[root.QposAddress + i] = frame[i];
        }

        var vFrame = motionFiles[clipIdx].Qvel(frameIdx);
        int nV = vFrame.Length;

        for (int i = 0; i < nV; i++)
        {
            MjScene.Instance.Data->qvel[root.DofAddress + i] = vFrame[i];
        }

        for (int i = 0; i < nV; i++)
        {
            MjScene.Instance.Data->qacc[root.DofAddress + i] = 0;
        }

        MujocoLib.mj_forward(MjScene.Instance.Model, MjScene.Instance.Data);
        MjScene.Instance.SyncUnityToMjState();
    }

    public void SetMjState() => SetMjState(CurClipIdx, CurFrameIndexInClip);

    public void SetMjState(int clipIdx, float time) => SetMjState(clipIdx, (int)(time / Time.fixedDeltaTime));

    public void SetMjStateNormalizedTime(int clipIdx, float normalizedTime) => SetMjStateNormalizedTime(clipIdx, (int)((motionFiles[clipIdx].Length - 1) * (normalizedTime%1)));

    public void PrintDatasets()
    {
        long fileId = H5F.open(filePath, H5F.ACC_RDONLY);
        H5O.iterate_t iterateCallback = (long loc_id, IntPtr namePtr, ref H5O.info_t info, IntPtr op_data) =>
        {
            byte[] nameBytes = new byte[2048]; // Adjust the size accordingly
            Marshal.Copy(namePtr, nameBytes, 0, nameBytes.Length);
            string objectName = System.Text.Encoding.ASCII.GetString(nameBytes).Split('\0')[0];

            // Check if the object is a dataset
            if (H5O.exists_by_name(loc_id, objectName, H5P.DEFAULT) > 0 && H5O.get_info_by_name(loc_id, objectName, ref info, H5P.DEFAULT) >= 0)
            {
                if (info.type == H5O.type_t.DATASET)
                {
                    long dataSetId = H5D.open(fileId, objectName);
                    long dspace = H5D.get_space(dataSetId);
                    int ndims = H5S.get_simple_extent_ndims(dspace);
                    ulong[] dims = new ulong[ndims];
                    H5S.get_simple_extent_dims(dspace, dims, null);
                    Debug.Log($"Dataset found: {objectName} of shape {string.Join(", ", dims)}");
                    H5D.close(dataSetId);
                }
            }

            return 0; // Continue iterating
        };

        // Iterate over objects in the file
        H5O.visit(fileId, H5.index_t.NAME, H5.iter_order_t.INC, iterateCallback, IntPtr.Zero);

        // Close the file
        H5F.close(fileId);
    }

    public void IncrementFrame()
    {
        CurFrameIndexInClip++;
        if(CurFrameIndexInClip >= motionFiles[CurClipIdx].Length)
        {
            CurFrameIndexInClip = 0;
            CurClipIdx++;
            ClipEndedEvent?.Invoke(this, new MocapPlaybackEventArgs(CurClipIdx-1, motionFiles[CurClipIdx-1].Length));
        }
        if(CurClipIdx >= motionFiles.Count)
        {
            CurClipIdx = 0;
            ClipsRanOutEvent?.Invoke(this, new MocapPlaybackEventArgs(motionFiles.Count-1, motionFiles[motionFiles.Count - 1].Length));
        }
    }

    [Serializable]
    internal class DataSubsetNameCollection
    {
        public List<string> names;
        public float mocap_dt;
    }


    public class CmuMotionData
    {
        readonly public string prefix;
        Dictionary<string, float[][]> fields;

        private int length;
        public int Length => length;

        public IEnumerable<string> Keys => fields.Keys;
        private float dt;
        private float fs;

        public CmuMotionData(long fileId, string prefix, IEnumerable<string> fieldNames, double mocapDt = -1)
        {
            this.prefix = prefix;
            dt = mocapDt > -1? (float)mocapDt : Time.fixedDeltaTime;
            fs = 1 / dt;
            fields = new Dictionary<string, float[][]>();
            length = 0;
            foreach (var fieldName in fieldNames)
            {
                var hdf5view = new DataFrameFieldView(fileId, prefix + fieldName);
                var data = hdf5view.GetArray();

                if (mocapDt != -1)
                {

                    var dataMatrix = Matrix<float>.Build.DenseOfRowArrays(data);

                    var mocapTime = Generate.LinearRange(0, mocapDt, dataMatrix.RowCount * mocapDt - 1e-8);
                    var newTime = Generate.LinearRange(0, Time.fixedDeltaTime, dataMatrix.RowCount * mocapDt);

                    var newData = new float[newTime.Length][];
                    for (int t = 0; t < newTime.Length; t++)
                    {
                        newData[t] = new float[dataMatrix.ColumnCount];
                    }
                    for (int r = 0; r < dataMatrix.ColumnCount; r++)
                    {
                        var interpolator = CubicSpline.InterpolateAkimaSorted(mocapTime, dataMatrix.Column(r).Select(x => (double)x).ToArray());
                        for (int t = 0; t < newTime.Length; t++)
                        {
                            newData[t][r] = (float)interpolator.Interpolate(newTime[t]);
                        }
                    }
                    data = newData;
                }

                fields[fieldName] = data;
                var fieldSeries = Transpose(fields[fieldName]).Select(f => f.ToArray()).ToArray();
                var newLength = fields[fieldName].Length - 2;  // Reserve two samples for second order approximations 
                if (length != 0 && length != newLength)
                {
                    Debug.LogError($"Inconsistent field lengths in dataset {prefix}!");
                }
                length = newLength;
            }
        }

        private static IEnumerable<IEnumerable<T>> Transpose<T>(IEnumerable<IEnumerable<T>> list)
        {
            return
                //generate the list of top-level indices of transposed list
                Enumerable.Range(0, list.First().Count())
                //selects elements at list[y][x] for each x, for each y
                .Select(x => list.Select(y => y.ElementAt(x)));
        }

        public float[][] this[string field]
        {
            get => fields[field];
        }

        public float[] Qpos(int timeIdx)
        {
            return fields["position"][timeIdx].Concat(fields["quaternion"][timeIdx])
                                              .Concat(fields["joints"][timeIdx])
                                              .ToArray();
        }

        public float[] BodyPos(int bodyId, int timeIdx)
        {
            return fields["body_positions"][timeIdx][(bodyId*3)..((bodyId+1)*3)];
        }

        public float[] BodyVel(int bodyId, int timeIdx)
        {
            return BodyPos(bodyId, timeIdx).Zip(BodyPos(bodyId, timeIdx+1), (cur, next) => (next-cur) * fs).ToArray();
        }

        public float[] BodyRot(int bodyId, int timeIdx)
        {
            return fields["body_quaternions"][timeIdx][(bodyId * 4)..((bodyId + 1) * 4)];
        }

        public float[] BodyAngularVel(int bodyId, int timeIdx)
        {
            return Utils.QuaternionError(BodyRot(bodyId, timeIdx+1), BodyRot(bodyId, timeIdx)).Select(q => q * fs).ToArray();
        }

        public float[] RootPos(int timeIdx)
        {
            return fields["position"][timeIdx];
        }

        public float[] RootRot(int timeIdx)
        {
            return fields["quaternion"][timeIdx];
        }

        public float[] RootVel(int timeIdx)
        {
            return fields["velocity"][timeIdx];
        }

        public float[] RootAngularVel(int timeIdx)
        {
            return fields["angular_velocity"][timeIdx];
        }

        public Matrix4x4 RootUnityTransform(int timeIdx)
        {
            var mjQuatArr = fields["quaternion"][timeIdx];
            var quat = new Quaternion(mjQuatArr[1], mjQuatArr[3], mjQuatArr[2], -mjQuatArr[0]);

            var mjPosArr = fields["position"][timeIdx];
            var pos = new Vector3(mjPosArr[0], mjPosArr[2], mjPosArr[1]);

            return Matrix4x4.TRS(pos, quat, Vector3.one);
        }

        public Matrix4x4 RootMjTransform(int timeIdx)
        {
            var mjQuatArr = fields["quaternion"][timeIdx];
            var quat = new Quaternion(mjQuatArr[1], mjQuatArr[2], mjQuatArr[3], mjQuatArr[0]);

            var mjPosArr = fields["position"][timeIdx];
            var pos = new Vector3(mjPosArr[0], mjPosArr[1], mjPosArr[2]);

            return Matrix4x4.TRS(pos, quat, Vector3.one);
        }

        public float[] Qvel(int timeIdx)
        {
            return fields["velocity"][timeIdx].Concat(fields["angular_velocity"][timeIdx])
                                              .Concat(fields["joints_velocity"][timeIdx])
                                              .ToArray();
        }

        private struct DataFrameFieldView
        {
            readonly long fileId;
            readonly string datasetPath;
            readonly ulong width;
            readonly ulong length;
            readonly bool isEmpty;


            public DataFrameFieldView(long fileId, string datasetPath)
            {
                long dataSetId = H5D.open(fileId, datasetPath);
                this.fileId = fileId;
                this.datasetPath = datasetPath;
                long dspace = H5D.get_space(dataSetId);
                int ndims = H5S.get_simple_extent_ndims(dspace);

                isEmpty = false;
                if (ndims != 2)
                {
                    if (ndims == -1)
                    {
                        Debug.LogWarning($"Dataset not found ({datasetPath})! Skipping dataset.");
                    }
                    Debug.LogWarning($"Non-2D field read ({datasetPath})! Skipping dataset.");
                    isEmpty = true;
                }

                ulong[] dims = new ulong[ndims];
                H5S.get_simple_extent_dims(dspace, dims, null);

                width = dims[0];
                length = dims[1];

                H5D.close(dataSetId);
            }


            public float[][] GetArray()
            {
                if (isEmpty)
                {
                    return new float[0][];
                }
                long fieldId = H5D.open(fileId, datasetPath);
                float[,] arr = new float[width, length];

                long typeId = H5D.get_type(fieldId);
                GCHandle gch = GCHandle.Alloc(arr, GCHandleType.Pinned);
                try
                {
                    H5D.read(fieldId, typeId, H5S.ALL, H5S.ALL, H5P.DEFAULT,
                             gch.AddrOfPinnedObject());
                }
                finally
                {
                    gch.Free();
                }


                float[][] arrOut = new float[length][];
                for (int i = 0; (ulong)i < length; i++)
                {
                    arrOut[i] = new float[width];
                    for (int j = 0; (ulong)j < width; j++)
                    {
                        arrOut[i][j] = arr[j, i];
                    }
                }
                H5D.close(fieldId);
                return arrOut;
            }

            public double[] this[int idx]
            {

                get
                {
                    if (isEmpty) return new double[0];
                    long fieldId = H5D.open(fileId, datasetPath);
                    double[] arr = new double[width];

                    // Define the hyperslab to select a single row
                    ulong[] start = { (ulong)idx, 0 }; // rowNumber is the row you want to read
                    ulong[] count = { 1, width }; // numberOfColumns is the width of your dataset

                    long rowSpace = H5S.create_simple(2, count, null);
                    long dataSpace = H5D.get_space(fieldId);
                    H5S.select_hyperslab(dataSpace, H5S.seloper_t.SET, start, null, count, null);

                    long typeId = H5D.get_type(fieldId);

                    GCHandle gch = GCHandle.Alloc(arr, GCHandleType.Pinned);
                    try
                    {
                        H5D.read(fieldId, typeId, rowSpace, dataSpace, H5P.DEFAULT,
                                    gch.AddrOfPinnedObject());
                    }
                    finally
                    {
                        gch.Free();
                    }
                    return arr;
                }
            }

        }
    }

    public class MocapPlaybackEventArgs: EventArgs
    {
        public readonly int clipId;
        public readonly int frameId;
        public MocapPlaybackEventArgs(int clipId, int frameId)
        {
            this.clipId = clipId;
            this.frameId = frameId;
        }
    }
}

