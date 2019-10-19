/*
 * Copyright (c) 2019, Sébastien GAGGINI AKA Sarbian, France
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

using System;
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

    private readonly bool additive;

    public enum Inputs
    {
        power = 0,

        density = 1,

        mach = 2,

        parttemp = 3,

        externaltemp = 4,

        time = 5
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

            minOutput = maxOutput = additive ? 0f : 1f;
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

                    minInput[i] = Mathf.Min(minInput[i], key);
                    maxInput[i] = Mathf.Max(maxInput[i], key);

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

                    minInput[i] = Mathf.Min(minInput[i], key);
                    maxInput[i] = Mathf.Max(maxInput[i], key);

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
                    ? result + logCurves[i].Value(Mathf.Log10(input))
                    : result * logCurves[i].Value(Mathf.Log10(input));
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
        MonoBehaviour.print("[SmokeScreen MultiInputCurve] " + s);
    }
}