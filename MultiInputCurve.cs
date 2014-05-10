using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using UnityEngine;

public class MultiInputCurve
{
    private string name;
    private Dictionary<Inputs, FXCurve> curves;
    private Dictionary<Inputs, FXCurve> logCurves;

    private bool hasLog = false;
    bool additive;

    public enum Inputs
    {
        power,
        density,
        mach,
        parttemp,
        externaltemp
    }

    public MultiInputCurve(string name, bool additive = false)
    {
        this.name = name;
        this.additive = additive;

        int count = Enum.GetValues(typeof(Inputs)).Length;

        curves = new Dictionary<Inputs, FXCurve>(count);
        logCurves = new Dictionary<Inputs, FXCurve>(count);

        foreach (Inputs key in Enum.GetValues(typeof(Inputs)))
        {
            curves[key] = new FXCurve(key.ToString(), additive ? 0f : 1f);
        }
    }

    public void Load(ConfigNode node)
    {
        // For backward compat load the power curve as the one with the same name
        // it will get overwritten if a power is defined in the subnode
        curves[Inputs.power].Load(name, node);

        if (node.HasNode(name))
        {
            foreach (Inputs key in Enum.GetValues(typeof(Inputs)))
            {
                curves[key].Load(key.ToString(), node.GetNode(name));

                string logKey = "log" + key;
                if (node.GetNode(name).HasValue(logKey))
                {
                    hasLog = true;
                    logCurves[key] = new FXCurve(logKey, additive ? 0f : 1f);
                    logCurves[key].Load(logKey, node.GetNode(name));
                }
            }
        }
    }

    public float Value(Dictionary<Inputs, float> inputs)
    {
        float result = additive ? 0f : 1f;

        foreach (Inputs key in Enum.GetValues(typeof(Inputs)))
        {
            float input = inputs[key];

            result = additive ? result + curves[key].Value(input) : result * curves[key].Value(input);

            FXCurve logCurve;
            if (hasLog && logCurves.TryGetValue(key, out logCurve))
            {
                result = additive ? result + logCurve.Value(Mathf.Log(input)) : result * logCurve.Value(Mathf.Log(input));
            }
        }
        return result;
    }

    public void Save(ConfigNode node)
    {
        foreach (FXCurve curve in curves.Values)
        {
            ConfigNode subNode = new ConfigNode(name);
            curve.Save(subNode);
            node.AddNode(subNode);
        }
        foreach (FXCurve curve in logCurves.Values)
        {
            ConfigNode subNode = new ConfigNode(name);
            curve.Save(subNode);
            node.AddNode(subNode);
        }
    }
}
