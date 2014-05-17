using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace SmokeScreen
{
    using UnityEngine;

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    internal class SmokeScreenConfig : MonoBehaviour
    {
        [Persistent]
        public int maximumActiveParticles = 8000; // The engine won't spawn more than 10k anyway

        [Persistent]
        public bool globalCollideDisable = false;

        [Persistent]
        public bool globalPhysicalDisable = false;

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

        SmokeScreenConfig()
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
                        particleDecimate = -activeParticles / Instance.maximumActiveParticles; // negative we keep each n
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

        void Start()
        {
            UrlDir.UrlConfig[] config = GameDatabase.Instance.GetConfigs("SmokeScreen");

            if (config.Length > 0)
            {
                MonoBehaviour.print("SmokeScreenConfig loading config");
                ConfigNode node = config[0].config;
                MonoBehaviour.print("SmokeScreenConfig  " + node.CountValues);
                MonoBehaviour.print("SmokeScreenConfig  " + node.values[0].name + " " + node.values[1].name + " " + node.values[2].name + " ");

                ConfigNode.LoadObjectFromConfig(this, node);
            }
            else
            {
                MonoBehaviour.print("SmokeScreenConfig could not load config");
            }
            lastHash = Hash();
        }

        void Update()
        {
            if (lastHash != Hash() && lastSave + 10f < Time.time )
            {
                lastHash = Hash();
                lastSave = Time.time;
                this.Save();
            }
        }

        private int Hash()
        {
            return string.Concat(globalCollideDisable, globalPhysicalDisable, maximumActiveParticles).GetHashCode();
        }

        void Save()
        {
            string path = System.IO.Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location) + "/SmokeScreen.cfg";

            MonoBehaviour.print("SmokeScreenConfig saving config in " + path);

            ConfigNode topNode = new ConfigNode("SmokeScreen");
            ConfigNode node = new ConfigNode("SmokeScreen");
            ConfigNode.CreateConfigFromObject(this, node);
            topNode.AddNode(node);
            topNode.Save(path);
        }

    }
}
