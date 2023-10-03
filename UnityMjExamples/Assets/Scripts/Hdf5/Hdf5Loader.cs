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
using Unity.Burst.Intrinsics;

public class Hdf5Loader : MonoBehaviour
{
    [SerializeField]
    string filePath;

    List<string> prefixes;

    [SerializeField]
    TextAsset subsetJson;

    [SerializeField]
    List<string> fields = new List<string> { "position", "quaternion", "joints", "velocity", "angular_velocity", "joints_velocity"};

    [SerializeField]
    MjFreeJoint root;

    public IReadOnlyList<CmuMotionData> motionFiles;

    public class CmuMotionData
    {
        readonly public string prefix;
        Dictionary<string, float[][]> fields;

        private int length;
        public int Length => length;

        public IEnumerable<string> Keys => fields.Keys;

        public CmuMotionData(long fileId, string prefix, IEnumerable<string> fieldNames, double mocapDt=-1)
        {
            this.prefix = prefix;
            fields = new Dictionary<string, float[][]>();
            length = 0;
            foreach(var fieldName in fieldNames) 
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
                    for (int r=0; r< dataMatrix.ColumnCount; r++)
                    {
                        var interpolator = CubicSpline.InterpolateAkimaSorted(mocapTime, dataMatrix.Column(r).Select(x => (double)x).ToArray());
                        for (int t=0; t<newTime.Length; t++)
                        {
                            newData[t][r] = (float) interpolator.Interpolate(newTime[t]);
                        }
                    }
                    data = newData;
                }

                fields[fieldName] = data;
                var fieldSeries = Transpose(fields[fieldName]).Select(f => f.ToArray()).ToArray();
                var newLength = fields[fieldName].Length;
                if (length != 0 && length != newLength) 
                {
                    Debug.LogError($"Inconsistent field lengths in dataset {prefix}!");
                }
                length = newLength;
            }
        }

        private static IEnumerable<IEnumerable<T>> Transpose<T>( IEnumerable<IEnumerable<T>> list)
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
                    long fieldId =H5D.open(fileId, datasetPath);
                    double[] arr = new double[width];

                    // Define the hyperslab to select a single row
                    ulong[] start = { (ulong) idx, 0 }; // rowNumber is the row you want to read
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

    // Start is called before the first frame update
    private void Start()
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
            Debug.Log(mf.prefix + ":" + mf.Keys.Count() +"; "+ mf[fields[0]].Length +", "+ mf[fields[0]][0].Length);
        }
        MjState.ExecuteAfterMjStart(SetMjState);
        MjScene.Instance.postUpdateEvent += (_, _) => SetMjState();
        
    }

    private unsafe void SetMjState()
    {
        int timeIdx = (int)(Time.fixedTime / Time.fixedDeltaTime);
        int fileIdx = 0;

        while(timeIdx >= motionFiles[fileIdx].Length)
        {
            timeIdx -= motionFiles[fileIdx].Length;
            fileIdx++;
            if (fileIdx >= motionFiles.Count) return;
        }

        var frame = motionFiles[fileIdx].Qpos(timeIdx);
        for (int i=0; i<frame.Length; i++)
        {
            MjScene.Instance.Data ->qpos[root.QposAddress + i] = frame[i];
        }

        var vFrame = motionFiles[fileIdx].Qvel(timeIdx);
        int nV = vFrame.Length;

        for (int i = 0; i < nV; i++)
        {
            MjScene.Instance.Data->qvel[root.DofAddress + i] = vFrame[i];
        }

        for (int i = 0; i < nV; i++)
        {
            MjScene.Instance.Data->qacc[root.DofAddress + i] = 0;
        }
    }

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

    [Serializable]
    internal class DataSubsetNameCollection
    {
        public List<string> names;
        public float mocap_dt;
    }
}

