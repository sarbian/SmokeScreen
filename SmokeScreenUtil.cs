/*
 * Copyright (c) 2014, Sébastien GAGGINI AKA Sarbian, France
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

namespace SmokeScreen
{
    using System;
    using System.Collections.Generic;
    using System.Text;
    using UnityEngine;

    internal static class SmokeScreenUtil
    {
        public static string WriteRootNode(ConfigNode node)
        {
            StringBuilder builder = new StringBuilder();

            //print("node.values.Count " + node.values.Count + " node.nodes.Count " + node.nodes.Count);
            for (int i = 0; i < node.values.Count; i++)
            {
                ConfigNode.Value item = node.values[i];
                builder.AppendLine(string.Concat(item.name, " = ", item.value));
            }
            for (int j = 0; j < node.nodes.Count; j++)
            {
                WriteNodeString(node.nodes[j], ref builder, string.Empty);
            }
            return builder.ToString();
        }

        public static void WriteNodeString(ConfigNode node, ref StringBuilder builder, string indent)
        {
            builder.AppendLine(string.Concat(indent, node.name));
            builder.AppendLine(string.Concat(indent, "{"));
            string str = string.Concat(indent, "  ");
            for (int i = 0; i < node.values.Count; i++)
            {
                ConfigNode.Value item = node.values[i];
                builder.AppendLine(string.Concat(str, item.name, " = ", item.value));
            }
            for (int j = 0; j < node.nodes.Count; j++)
            {
                WriteNodeString(node, ref builder, str);
            }
            builder.AppendLine(string.Concat(indent, "}"));
        }

        public static ConfigNode RecurseFormat(List<string[]> cfg)
        {
            int num = 0;
            ConfigNode configNode = new ConfigNode("root");
            RecurseFormat(cfg, ref num, configNode);
            return configNode;
        }

        public static void RecurseFormat(List<string[]> cfg, ref int index, ConfigNode node)
        {
            while (index < cfg.Count)
            {
                if (cfg[index].Length == 2)
                {
                    node.values.Add(new ConfigNode.Value(cfg[index][0], cfg[index][1]));
                    index = index + 1;
                }
                else if (cfg[index][0] != "{")
                {
                    if (cfg[index][0] == "}")
                    {
                        index = index + 1;
                        return;
                    }
                    if (!NextLineIsOpenBrace(cfg, index))
                    {
                        index = index + 1;
                    }
                    else
                    {
                        ConfigNode configNode = new ConfigNode(cfg[index][0]);
                        node.nodes.Add(configNode);
                        index = index + 2;
                        RecurseFormat(cfg, ref index, configNode);
                    }
                }
                else
                {
                    ConfigNode configNode1 = new ConfigNode(string.Empty);
                    node.nodes.Add(configNode1);
                    index = index + 1;
                    RecurseFormat(cfg, ref index, configNode1);
                }
            }
        }

        public static bool NextLineIsOpenBrace(List<string[]> cfg, int index)
        {
            int num = index + 1;
            if (num < cfg.Count && cfg[num].Length == 1 && cfg[num][0] == "{")
            {
                return true;
            }
            return false;
        }

        public static List<string[]> PreFormatConfig(string[] cfgData)
        {
            if (cfgData == null || cfgData.Length < 1)
            {
                Debug.LogError("Error: Empty part config file");
                return null;
            }
            List<string> strs = new List<string>(cfgData);
            int count = strs.Count;
            while (true)
            {
                int num = count - 1;
                count = num;
                if (num < 0)
                {
                    break;
                }
                strs[count] = strs[count];
                int num1 = strs[count].IndexOf("//");
                int num2 = num1;
                if (num1 != -1)
                {
                    if (num2 != 0)
                    {
                        strs[count] = strs[count].Remove(num2);
                    }
                    else
                    {
                        strs.RemoveAt(count);
                        continue;
                    }
                }
                strs[count] = strs[count].Trim();
                if (strs[count].Length != 0)
                {
                    int num3 = strs[count].IndexOf("}", 0);
                    num2 = num3;
                    if (num3 == -1 || num2 == 0 && strs[count].Length == 1)
                    {
                        int num4 = strs[count].IndexOf("{", 0);
                        num2 = num4;
                        if (num4 != -1 && (num2 != 0 || strs[count].Length != 1))
                        {
                            if (num2 > 0)
                            {
                                strs.Insert(count, strs[count].Substring(0, num2));
                                count++;
                                strs[count] = strs[count].Substring(num2);
                                num2 = 0;
                            }
                            if (num2 < strs[count].Length - 1)
                            {
                                strs.Insert(count + 1, strs[count].Substring(num2 + 1));
                                strs[count] = "{";
                                count = count + 2;
                            }
                        }
                    }
                    else
                    {
                        if (num2 > 0)
                        {
                            strs.Insert(count, strs[count].Substring(0, num2));
                            count++;
                            strs[count] = strs[count].Substring(num2);
                            num2 = 0;
                        }
                        if (num2 < strs[count].Length - 1)
                        {
                            strs.Insert(count + 1, strs[count].Substring(num2 + 1));
                            strs[count] = "}";
                            count = count + 2;
                        }
                    }
                }
                else
                {
                    strs.RemoveAt(count);
                }
            }
            List<string[]> strArrays = new List<string[]>(strs.Count);
            for (int i = 0; i < strs.Count; i++)
            {
                string item = strs[i];
                string[] strArrays1 = item.Split(new[] { '=' }, 2, StringSplitOptions.None);
                if (strArrays1.Length != 0)
                {
                    for (int j = 0; j < strArrays1.Length; j++)
                    {
                        strArrays1[j] = strArrays1[j].Trim();
                    }
                    strArrays.Add(strArrays1);
                }
            }
            return strArrays;
        }

        // The whole pad object is named "ksp_pad_launchPad"
        public const string LaunchPadGrateColliderName = "Launch Pad Grate";

        private const string LaunchPadColliderName = "LaunchPadColliderSmokeScreen";

        public static bool AddLaunchPadColliders(RaycastHit hit)
        {
            // the Grate Collider size is  (37.70, 20.22, 3.47). Way larger that the actual grate
            // The current collider do not cover all this area. More are needed

            Transform parentTransform = hit.collider.gameObject.transform;

            ////print("AddLaunchPadColliders col name = " + hit.collider.gameObject.name);

            ////print("AddLaunchPadColliders parent col name = " + hit.collider.gameObject.transform.parent.gameObject.name);

            // Are the collider already here ?
            if (parentTransform.FindChild(LaunchPadColliderName))
            {
                return true;
            }

            GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = LaunchPadColliderName;
            cube.renderer.material.color = Color.green;
            cube.transform.parent = parentTransform;
            cube.transform.localPosition = new Vector3(8.5f, 0, 2.3f);
            cube.transform.localRotation = parentTransform.localRotation;
            cube.transform.localScale = new Vector3(0.1f, 7, 16);

            cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = LaunchPadColliderName;
            cube.renderer.material.color = Color.green;
            cube.transform.parent = parentTransform;
            cube.transform.localPosition = new Vector3(7, 10.5f, 2.3f);
            cube.transform.localRotation = parentTransform.localRotation * Quaternion.Euler(0, 60, 0);
            cube.transform.localScale = new Vector3(7f, 7, 0.1f);

            cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = LaunchPadColliderName;
            cube.renderer.material.color = Color.green;
            cube.transform.parent = parentTransform;
            cube.transform.localPosition = new Vector3(7, -10.5f, 2.3f);
            cube.transform.localRotation = parentTransform.localRotation * Quaternion.Euler(0, -60, 0);
            cube.transform.localScale = new Vector3(7f, 7, 0.1f);

            cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = LaunchPadColliderName;
            cube.renderer.material.color = Color.green;
            cube.transform.parent = parentTransform;
            cube.transform.localPosition = new Vector3(-8.5f, 0, 2.3f);
            cube.transform.localRotation = parentTransform.localRotation;
            cube.transform.localScale = new Vector3(0.1f, 7, 16);

            cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = LaunchPadColliderName;
            cube.renderer.material.color = Color.green;
            cube.transform.parent = parentTransform;
            cube.transform.localPosition = new Vector3(-7, 10.5f, 2.3f);
            cube.transform.localRotation = parentTransform.localRotation * Quaternion.Euler(0, -60, 0);
            cube.transform.localScale = new Vector3(7f, 7, 0.1f);

            cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = LaunchPadColliderName;
            cube.renderer.material.color = Color.green;
            cube.transform.parent = parentTransform;
            cube.transform.localPosition = new Vector3(-7, -10.5f, 2.3f);
            cube.transform.localRotation = parentTransform.localRotation * Quaternion.Euler(0, 60, 0);
            cube.transform.localScale = new Vector3(7f, 7, 0.1f);

            return true;
        }

        private static void print(String s)
        {
            MonoBehaviour.print("[SmokeScreen SmokeScreenUtil] " + s);
        }
    }
}