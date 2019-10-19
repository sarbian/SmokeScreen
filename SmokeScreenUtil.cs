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

namespace SmokeScreen
{
    using System;
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
            if (parentTransform.FindDeepChild(LaunchPadColliderName))
            {
                return true;
            }

            GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = LaunchPadColliderName;
            cube.GetComponent<Renderer>().material.color = Color.green;
            cube.transform.parent = parentTransform;
            cube.transform.localPosition = new Vector3(8.5f, 0, 2.3f);
            cube.transform.localRotation = parentTransform.localRotation;
            cube.transform.localScale = new Vector3(0.1f, 7, 16);

            cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = LaunchPadColliderName;
            cube.GetComponent<Renderer>().material.color = Color.green;
            cube.transform.parent = parentTransform;
            cube.transform.localPosition = new Vector3(7, 10.5f, 2.3f);
            cube.transform.localRotation = parentTransform.localRotation * Quaternion.Euler(0, 60, 0);
            cube.transform.localScale = new Vector3(7f, 7, 0.1f);

            cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = LaunchPadColliderName;
            cube.GetComponent<Renderer>().material.color = Color.green;
            cube.transform.parent = parentTransform;
            cube.transform.localPosition = new Vector3(7, -10.5f, 2.3f);
            cube.transform.localRotation = parentTransform.localRotation * Quaternion.Euler(0, -60, 0);
            cube.transform.localScale = new Vector3(7f, 7, 0.1f);

            cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = LaunchPadColliderName;
            cube.GetComponent<Renderer>().material.color = Color.green;
            cube.transform.parent = parentTransform;
            cube.transform.localPosition = new Vector3(-8.5f, 0, 2.3f);
            cube.transform.localRotation = parentTransform.localRotation;
            cube.transform.localScale = new Vector3(0.1f, 7, 16);

            cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = LaunchPadColliderName;
            cube.GetComponent<Renderer>().material.color = Color.green;
            cube.transform.parent = parentTransform;
            cube.transform.localPosition = new Vector3(-7, 10.5f, 2.3f);
            cube.transform.localRotation = parentTransform.localRotation * Quaternion.Euler(0, -60, 0);
            cube.transform.localScale = new Vector3(7f, 7, 0.1f);

            cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = LaunchPadColliderName;
            cube.GetComponent<Renderer>().material.color = Color.green;
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