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

public class Hdf5Loader : MonoBehaviour
{
    [SerializeField]
    string filePath;

    [SerializeField]
    List<string> prefixes;

    [SerializeField]
    List<string> fields;

    [SerializeField]
    MjFreeJoint root;

    Dictionary<string, Dictionary<string, float[][]>> datasets;

    // Start is called before the first frame update
    private void Start()
    {
        datasets = new Dictionary<string, Dictionary<string, float[][]>>();

        long fileId = H5F.open(filePath, H5F.ACC_RDONLY);
        foreach(var prefix in prefixes)
        {
            foreach(var field in fields)
            {
                long dataSetId = H5D.open(fileId, prefix+field);
                long dspace = H5D.get_space(dataSetId);
                int ndims = H5S.get_simple_extent_ndims(dspace);

                if (ndims != 2)
                {
                    if (ndims == -1)
                    {
                        Debug.LogWarning($"Dataset not found ({prefix + field})! Skipping dataset.");
                        continue;
                    }
                    Debug.LogWarning($"Non-2D field read ({prefix+field})! Skipping dataset.");
                    continue;
                }

                ulong[] dims = new ulong[ndims];
                H5S.get_simple_extent_dims(dspace, dims, null);
                double[,] arr = new double[dims[0], dims[1]];

                long typeId = H5D.get_type(dataSetId);
                GCHandle gch = GCHandle.Alloc(arr, GCHandleType.Pinned);
                try
                {
                    H5D.read(dataSetId, typeId, H5S.ALL, H5S.ALL, H5P.DEFAULT,
                             gch.AddrOfPinnedObject());
                }
                finally
                {
                    gch.Free();
                }

                float[][] floatArr = new float[dims[1]][];

                for (int i=0; (ulong)i <  dims[1]; i++)
                {
                    floatArr[i] = new float[dims[0]];
                    for (int j=0; (ulong)j < dims[0]; j++)
                    {
                        floatArr[i][j] = (float)arr[j, i];
                    }
                }

                var hmm = arr.Cast<float[,]>();
                if(!datasets.ContainsKey(prefix))
                {
                    datasets.Add(prefix, new Dictionary<string, float[][]>());
                }
                datasets[prefix].Add(field, floatArr);
                H5D.close(dataSetId);
            }
        }
        H5F.close(fileId);
        foreach(var kv in datasets)
        {
            Debug.Log(kv.Key + ":" + kv.Value.Count +"; "+ kv.Value[fields[0]].Length +", "+ kv.Value[fields[0]][0].Length);
        }
        MjState.ExecuteAfterMjStart(SetMjState);
        MjScene.Instance.ctrlCallback += (_, _) => SetMjState();
        
    }

    private unsafe  void SetMjState()
    {
        
        var frame = datasets[prefixes[^1]][fields[0]][0];
        for (int i=0; i<frame.Length; i++)
        {
            MjScene.Instance.Data ->qpos[i] = frame[i];
        }

        frame = datasets[prefixes[^1]][fields[1]][0];
        for (int i = 0; i < frame.Length; i++)
        {
            MjScene.Instance.Data->qvel[i] = frame[i];
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
}

