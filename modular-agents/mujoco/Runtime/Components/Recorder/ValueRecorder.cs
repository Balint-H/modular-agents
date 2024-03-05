using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;
using System.IO;
using ModularAgents;

namespace ModularAgentsRecorder
{
    public class ValueRecorder : MonoBehaviour
    {
        public enum shouldSaveData
        { 
            assetsfolder = 0,
            permanentpath = 1,
            no_saving = -1
        
        }


        [SerializeField]
        shouldSaveData saveIn ;
        [SerializeField]
        string fileName;

      
        /*
        public enum SaveFormat
        {
            CSV,
            JSON
        }
        */

        public EventHandler OnExport;

        //public SaveFormat format = SaveFormat.CSV;

        public string Header => string.Join(",", columnNames);

        public int Width => Header.Split(',').Length;

        protected List<List<float>> columns;

        protected List<string> columnNames;

        private void Awake()
        {
            columnNames = new List<string>();
            columns = new List<List<float>>();
        }

        protected IEnumerable<IEnumerable<float>> Rows => columns.Transpose();
        public void AddColumn(string colName)
        {
            columnNames.Add(colName);
            columns.Add(new List<float>());
        }

        void RecordValue(float value, string colName)
        {
           

            int colIdx = columnNames.IndexOf(colName);
            if (colIdx == -1)
            {
                AddColumn(colName);
                colIdx = columnNames.Count - 1;
            }

            columns[colIdx].Add(value);
            
        }

        public void Record(Vector3 vector, string colName)
        {
          

            var values = vector.GetComponents();
            var colNames = new[] { "_x", "_y", "_z" }.Select(d => colName + d);

            foreach ((var val, var name) in values.Zip(colNames, Tuple.Create))
            {
                RecordValue(val, name);
            }
        }
        
        public void Record(Vector2 vector, string colName)
        {
           

            var values = new float[] {vector.x, vector.y };
            var colNames = new[] { "_x", "_y", "_z" }.Select(d => colName + d);

            foreach ((var val, var name) in values.Zip(colNames, Tuple.Create))
            {
                RecordValue(val, name);
            }
        }

        public void Record(Quaternion quat, string colName)
        {
           
            var values = new[] { quat.x, quat.y, quat.z, quat.w };
            var colNames = new[] { "_x", "_y", "_z", "_w" }.Select(d => colName + d);
          

            foreach ((var val, var name) in values.Zip(colNames, Tuple.Create))
            {
                RecordValue(val, name);
            }
        }

        public void Record(IEnumerable<float> values, string colName)
        {
          
            int count = values.Count();
            IEnumerable<string> colNames;
            switch(count)
            {
                case 7:
                    colNames = new[] { "_x", "_y", "_z", "_rw", "_rx", "_ry", "_rz" }.Select(d => colName + d);
                    break;
                case 6:
                    colNames = new[] { "_x", "_y", "_z", "_rx", "_ry", "_rz" }.Select(d => colName + d);
                    break;
                case 4:
                    colNames = new[] { "_w", "_x", "_y", "_z"}.Select(d => colName + d);
                    break;
                case 3:
                    colNames = new[] { "_x", "_y", "_z"}.Select(d => colName + d);
                    break;
                default:
                    colNames = new[] { colName };
                    break;
            }
         

            foreach ((var val, var name) in values.Zip(colNames, Tuple.Create))
            {
                RecordValue(val, name);
            }
        }

        public void Record(double[] values, string colName)
        {
         


            int count = values.Count();
            IEnumerable<string> colNames;
            switch (count)
            {
                case 7:
                    colNames = new[] { "_x", "_y", "_z", "_rw", "_rx", "_ry", "_rz" }.Select(d => colName + d);
                    break;
                case 6:
                    colNames = new[] { "_x", "_y", "_z", "_rx", "_ry", "_rz" }.Select(d => colName + d);
                    break;
                case 4:
                    colNames = new[] { "_w", "_x", "_y", "_z" }.Select(d => colName + d);
                    break;
                case 3:
                    colNames = new[] { "_x", "_y", "_z" }.Select(d => colName + d);
                    break;
                default:
                    colNames = new[] { colName };
                    break;
            }

        

            foreach ((var val, var name) in values.Zip(colNames, Tuple.Create))
            {
                RecordValue((float) val, name);
            }


        }






        private void OnApplicationQuit()
        {
           // if (!shouldSave) return;

            /*
            switch (format)
            {
                case SaveFormat.CSV:
                    {

                        Debug.Log("Movement data saved at: " + Path.Combine(Application.persistentDataPath, fileName) + ".csv");
                        File.WriteAllLines(Path.Combine(Application.persistentDataPath, fileName)+".csv", Rows.Select(l => string.Join(",", l)).Prepend(Header));
                        break;


                    }
                case SaveFormat.JSON:
                    throw new NotImplementedException();
            }*/


            switch (saveIn)
            {
                case shouldSaveData.assetsfolder :
                    {

                        Debug.Log("Movement data saved at: " + Path.Combine(Application.dataPath, fileName) + ".csv");
                        File.WriteAllLines(Path.Combine(Application.dataPath, fileName) + ".csv", Rows.Select(l => string.Join(",", l)).Prepend(Header));
                        break;


                    }
                case shouldSaveData.permanentpath:
                    {
                        Debug.Log("Movement data saved at: " + Path.Combine(Application.persistentDataPath, fileName) + ".csv");
                        File.WriteAllLines(Path.Combine(Application.persistentDataPath, fileName) + ".csv", Rows.Select(l => string.Join(",", l)).Prepend(Header));
                        break;



                    }


                    throw new NotImplementedException();
            }



            OnExport?.Invoke(this, EventArgs.Empty);

        }
    }
}