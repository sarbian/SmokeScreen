﻿/*
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

using System.IO;
using System.Reflection;
using UnityEngine;

namespace SmokeScreen
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    internal class SmokeScreenConfig : MonoBehaviour
    {
        [Persistent] public int maximumActiveParticles = 8000; // The engine won't spawn more than 10k anyway

        [Persistent] public bool globalCollideDisable = false;

        [Persistent] public bool globalPhysicalDisable = false;

        public static int activeParticles = 0;

        public static int particleDecimate = 0;

        public static uint particleCounter = 0;

        // Particles Emitter with more than decimateFloor particles will have
        // some particles culled if there is more than maximumActiveParticles active
        public static int decimateFloor = 30;

        private static float lastTime = 0;

        private int lastHash = 0;

        private float lastSave = 0;

        public static SmokeScreenConfig Instance;

        private SmokeScreenConfig()
        {
            Instance = this;
        }

        public static void UpdateParticlesCount()
        {
            if (lastTime < Time.fixedTime)
            {
                if (activeParticles > Instance.maximumActiveParticles)
                {
                    int toRemove = activeParticles - Instance.maximumActiveParticles;
                    if (toRemove < Instance.maximumActiveParticles)
                    {
                        particleDecimate = activeParticles / (toRemove + 1); // positive we remove each n
                    }
                    else
                    {
                        particleDecimate = -activeParticles / Instance.maximumActiveParticles;

                        // negative we keep each n
                    }
                }
                else
                {
                    particleDecimate = 0;
                }

                activeParticles = 0;
                lastTime = Time.fixedTime;
            }
        }

        private void Start()
        {
            UrlDir.UrlConfig[] config = GameDatabase.Instance.GetConfigs("SmokeScreen");

            if (config.Length > 0)
            {
                print("SmokeScreenConfig loading config");
                ConfigNode node = config[0].config;
                ConfigNode.LoadObjectFromConfig(this, node);
            }
            else
            {
                print("SmokeScreenConfig could not load config");
            }
            lastHash = Hash();
        }

        private void Update()
        {
            if (lastHash != Hash() && lastSave + 10f < Time.time)
            {
                lastHash = Hash();
                lastSave = Time.time;
                Save();
            }
        }

        private int Hash()
        {
            return string.Concat(globalCollideDisable, globalPhysicalDisable, maximumActiveParticles).GetHashCode();
        }

        private void Save()
        {
            string path = Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location)
                          + "/SmokeScreen.cfg";

            print("SmokeScreenConfig saving config");

            ConfigNode topNode = new ConfigNode("SmokeScreen");
            ConfigNode node = new ConfigNode("SmokeScreen");
            ConfigNode.CreateConfigFromObject(this, node);
            topNode.AddNode(node);
            topNode.Save(path);
        }
    }
}