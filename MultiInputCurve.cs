using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;



public class MultiInputCurve
{
    private string name;
    string[] keyNames;
    private Dictionary<string, FXCurve> curves;
    bool additive;

    public MultiInputCurve(string name, string[] keyNames, bool additive = false)
    {
        this.name = name;
        this.keyNames = keyNames;
        this.additive = additive;

        curves = new Dictionary<string, FXCurve>(keyNames.Length, StringComparer.Ordinal);

        foreach (string key in keyNames)
        {
            curves[key] = new FXCurve(key, additive ? 0f : 1f);
        }
    }

    public void Load(ConfigNode node)
    {
        // For backward compat load the power curve as the one with the same name
        // it will get overwritten if a power is defined in the subnode
        curves["power"].Load(name, node);

        if (node.HasNode(name))
            foreach (string key in keyNames)
                curves[key].Load(key, node.GetNode(name));
    }

    public float Value(Dictionary<string, float> input)
    {
        float result = additive ? 0f : 1f;

        foreach (string key in keyNames)
            if (additive)
                result += curves[key].Value(input[key]);
            else
                result *= curves[key].Value(input[key]);

        return result;
    }

    public void Save(ConfigNode node)
    {
        if (node.HasNode(name))
            foreach (string key in keyNames)
            {
                ConfigNode subNode = new ConfigNode(name);
                curves[key].Save(subNode);
                node.AddNode(subNode);
            }
    }

}
