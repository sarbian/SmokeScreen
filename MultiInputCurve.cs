using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using UnityEngine;

public class MultiInputCurve
{
    private string name;
    private FXCurve[] curves;
    private FXCurve[] logCurves;

    bool additive;

    public enum Inputs
    {
        power = 0,
        density = 1,
        mach = 2,
        parttemp = 3,
        externaltemp = 4
    }

    public static readonly int inputsCount = Enum.GetValues(typeof(Inputs)).Length;

    public MultiInputCurve(string name, bool additive = false)
    {
        this.name = name;
        this.additive = additive;
        
        curves = new FXCurve[inputsCount];
        logCurves = new FXCurve[inputsCount];

        for (int i = 0; i < inputsCount; i++)
        {
            curves[i] = new FXCurve(Enum.GetName(typeof(Inputs), i), additive ? 0f : 1f);
        }
    }

    public void Load(ConfigNode node)
    {
        // For backward compat load the power curve as the one with the same name
        // it will get overwritten if a power is defined in the subnode
        curves[(int)Inputs.power].Load(name, node);

        if (node.HasNode(name))
        {
            for (int i = 0; i < inputsCount; i++)
            {
                string key = Enum.GetName(typeof(Inputs), i);
                curves[i].Load(key, node.GetNode(name));

                string logKey = "log" + key;
                if (node.GetNode(name).HasValue(logKey))
                {
                    logCurves[i] = new FXCurve(logKey, additive ? 0f : 1f);
                    logCurves[i].Load(logKey, node.GetNode(name));
                }
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
                result = additive ? result + logCurves[i].Value(Mathf.Log(input)) : result * logCurves[i].Value(Mathf.Log(input));
            }
        }
        return result;
    }

    public void Save(ConfigNode node)
    {
        for (int i = 0; i < inputsCount; i++)
        {
            ConfigNode subNode = new ConfigNode(name);
            curves[i].Save(subNode);
            node.AddNode(subNode);
        }
        for (int i = 0; i < inputsCount; i++)
        {
            if (logCurves[i] != null)
            {
                ConfigNode subNode = new ConfigNode(name);
                logCurves[i].Save(subNode);
                node.AddNode(subNode);
            }
        }
    }
}
