using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;



public class MultiInputCurve
{
    private string name;
    private Dictionary<Input, FXCurve> curves;
    bool additive;

    public enum Input
    {
        power = 0,
        density,
        mach
    }

    public MultiInputCurve(string name, bool additive = false)
    {
        this.name = name;
        this.additive = additive;

        int count = Enum.GetValues(typeof(Input)).Length;

        curves = new Dictionary<Input, FXCurve>(count);

        foreach (Input key in Enum.GetValues(typeof(Input)))
        {
            curves[key] = new FXCurve(key.ToString(), additive ? 0f : 1f);
        }
    }

    public void Load(ConfigNode node)
    {
        // For backward compat load the power curve as the one with the same name
        // it will get overwritten if a power is defined in the subnode
        curves[Input.power].Load(name, node);

        if (node.HasNode(name))
        {
            foreach (Input key in Enum.GetValues(typeof(Input)))
            {
                curves[key].Load(key.ToString(), node.GetNode(name));
            }
        }
    }

    public float Value(Dictionary<Input, float> inputs)
    {
        float result = additive ? 0f : 1f;

        foreach (Input key in Enum.GetValues(typeof(Input)))
            if (additive)
                result += curves[key].Value(inputs[key]);
            else
                result *= curves[key].Value(inputs[key]);

        return result;
    }

    public void Save(ConfigNode node)
    {
        if (node.HasNode(name))
            foreach (Input key in Enum.GetValues(typeof(Input)))
            {
                ConfigNode subNode = new ConfigNode(name);
                curves[key].Save(subNode);
                node.AddNode(subNode);
            }
    }

}
