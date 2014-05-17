using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using SmokeScreen;
using UnityEngine;


// This class Serialization DOES NOT WORK in Unity
// Life would be so easier if it did.
[Serializable]
public class MultiInputCurve
{
    public string name;

    public FXCurve[] curves = new FXCurve[inputsCount];

    public FXCurve[] logCurves = new FXCurve[inputsCount];

    public float[] minInput = new float[inputsCount];

    public float[] maxInput = new float[inputsCount];

    public float minOutput;

    public float maxOutput;

    private bool additive;

    public enum Inputs
    {
        power = 0,

        density = 1,

        mach = 2,

        parttemp = 3,

        externaltemp = 4
    }

    public static readonly int inputsCount = Enum.GetValues(typeof(Inputs)).Length;

    //public const int inputsCount = 5;

    public MultiInputCurve(string name, bool additive = false)
    {
        //print("Constructor for " + name + " backup_node is null = " + (node_backup == null).ToString());
        this.name = name;
        this.additive = additive;
        //if (node_backup != null)
        //    print("node_backup is\n " + node_backup.Replace(Environment.NewLine, Environment.NewLine + "MultiInputCurve "));
    }

    private void Reset()
    {
        //print("Reset");
        for (int i = 0; i < inputsCount; i++)
        {
            string key = Enum.GetName(typeof(Inputs), i);
            //print("Resetting " + key);
            curves[i] = new FXCurve(key, additive ? 0f : 1f) { valueName = key };

            // FXCurve constructor does not set the value name

            this.minOutput = this.maxOutput = additive ? 0f : 1f;
        }
    }

    public void Test()
    {
        //if (!isLoaded) Restore();
        if (curves != null && curves[0] != null)
        {
            print("Test curve[0] is " + curves[0].valueName);
        }
        //print("Test for " + name + " backup_node is null = " + (backup_node == null).ToString());
    }

    public void Load(ConfigNode node)
    {
        Reset();
        // For backward compat load the power curve as the one with the same name
        // it will get overwritten if a power is defined in the subnode
        curves[(int)Inputs.power].Load(name, node);
        curves[(int)Inputs.power].valueName = "power";

        //print("Load of " + name);

        if (node.HasNode(name))
        {
            //print("Load HasNode " + name);
            for (int i = 0; i < inputsCount; i++)
            {
                string key = Enum.GetName(typeof(Inputs), i);
                //print("Loading " + key);
                curves[i].Load(key, node.GetNode(name));
                //print(
                //    "Loaded " + key + " in " + curves[i].valueName + " " + curves[i].keyFrames.Count() + " should be "
                //    + node.GetNode(name).GetValues(key).Length);

                string logKey = "log" + key;
                if (node.GetNode(name).HasValue(logKey))
                {
                    logCurves[i] = new FXCurve(logKey, additive ? 0f : 1f) { valueName = logKey };
                    // FXCurve constructor does not set the value name
                    logCurves[i].Load(logKey, node.GetNode(name));
                }
            }
        }
        UpdateMinMax();
        //isLoaded = true;
    }

    private void UpdateMinMax()
    {
        for (int i = 0; i < inputsCount; i++)
        {
            float minValue = additive ? 0f : 1f;
            float maxValue = additive ? 0f : 1f;

            if (!curves[i].evalSingle)
            {
                //print("UpdateMinMax i=" + i + " " + curves[i].fCurve.length);
                for (int j = 0; j < curves[i].fCurve.length; j++)
                {
                    float key = curves[i].fCurve.keys[j].time;
                    float val = curves[i].fCurve.keys[j].value;

                    this.minInput[i] = Mathf.Min(this.minInput[i], key);
                    this.maxInput[i] = Mathf.Max(this.maxInput[i], key);

                    minValue = Mathf.Min(minValue, val);
                    maxValue = Mathf.Max(maxValue, val);
                }
            }

            if (logCurves[i] != null && !logCurves[i].evalSingle)
            {
                //print("UpdateMinMax i=" + i + " " + logCurves[i].fCurve.length);
                for (int j = 0; j < logCurves[i].fCurve.length; j++)
                {
                    float key = logCurves[i].fCurve.keys[j].time;
                    float val = logCurves[i].fCurve.keys[j].value;

                    this.minInput[i] = Mathf.Min(this.minInput[i], key);
                    this.maxInput[i] = Mathf.Max(this.maxInput[i], key);

                    minValue = Mathf.Min(minValue, val);
                    maxValue = Mathf.Max(maxValue, val);
                }
            }
            if (additive)
            {
                minOutput += minValue;
            }
            else
            {
                maxOutput *= maxValue;
            }
        }
    }

    public float Value(float[] inputs)
    {
        float result = additive ? 0f : 1f;

        for (int i = 0; i < inputsCount; i++)
        {
            float input = inputs[i];

            result = additive ? result + curves[i].Value(input) : result * curves[i].Value(input);

            if (logCurves[i] != null)
            {
                result = additive
                             ? result + logCurves[i].Value(Mathf.Log(input))
                             : result * logCurves[i].Value(Mathf.Log(input));
            }
        }
        return result;
    }

    public void Save(ConfigNode node)
    {
        ConfigNode subNode = new ConfigNode(name);
        for (int i = 0; i < inputsCount; i++)
        {
            //print("Saving curve " + curves[i].valueName + " " + curves[i].keyFrames.Count());
            curves[i].Save(subNode);
            if (logCurves[i] != null)
            {
                //print("Saving curve " + logCurves[i].valueName);
                logCurves[i].Save(subNode);
            }
        }
        node.AddNode(subNode);
    }

    private static void print(String s)
    {
        MonoBehaviour.print("[MultiInputCurve] " + s);
    }
}
