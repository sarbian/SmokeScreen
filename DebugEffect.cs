using System;

namespace SmokeScreen
{
    [EffectDefinition("DEBUG_EFFECT")]
    class DebugEffect : EffectBehaviour
    {
        public override void OnEvent()
        {
            Print(effectName.PadRight(16) + "OnEvent single -------------------------------------------------------");
        }

        private float lastPower = -1;

        public override void OnEvent(float power)
        {
            if (Math.Abs(lastPower - power) > 0.01f)
            {
                lastPower = power;
                Print(effectName.PadRight(16)  + " " + instanceName + "OnEvent pow = " + power.ToString("F2"));
            }
        }

        public override void OnInitialize()
        {
            Print("OnInitialize");
        }

        public override void OnLoad(ConfigNode node)
        {
            Print("OnLoad");
        }

        public override void OnSave(ConfigNode node)
        {
            Print("OnSave");
        }

        private static void Print(String s)
        {
            print("[SmokeScreen DebugEffect] " + s);
        }
    }
}
