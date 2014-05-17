using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace SmokeScreen
{
    using UnityEngine;

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    class SmokeScreenUI : MonoBehaviour
    {
        private const string icon = "SmokeScreen/SmokeScreen";

        private IButton button;

        private bool showUI = false;

        private Rect winPos = new Rect(450, 50, 400, 100);

        private int winID = 512099;

        private SmokeScreenUI()
        {
            if (!ToolbarManager.ToolbarAvailable)
            {
                return;
            }

            button = ToolbarManager.Instance.add("SmokeScreen", "main");
            button.TexturePath = icon;
            button.ToolTip = "Toggle This Button's Icon";
            button.OnClick += (e) =>
                {
                    this.showUI = !this.showUI;
                };
        }

        private void OnDestroy()
        {
            button.Destroy();
        }

        private void OnGUI()
        {
            if (!HighLogic.LoadedSceneIsFlight)
            {
                return;
            }

            if (showUI)
            {
                winPos = GUILayout.Window(winID, winPos, windowGUI, "SmokeScreen", GUILayout.MinWidth(300));
            }
        }

        private void windowGUI(int ID)
        {
            GUILayout.BeginVertical();

            SmokeScreenConfig.Instance.globalCollideDisable = GUILayout.Toggle(SmokeScreenConfig.Instance.globalCollideDisable, "Globally disable Collide");
            SmokeScreenConfig.Instance.globalPhysicalDisable = GUILayout.Toggle(SmokeScreenConfig.Instance.globalPhysicalDisable, "Globally disable Physical");

            GUILayout.Space(10);

            GUILayout.BeginHorizontal();
            GUILayout.Label("maximumActiveParticles", GUILayout.ExpandWidth(true));
            int.TryParse(
                GUILayout.TextField(SmokeScreenConfig.Instance.maximumActiveParticles.ToString(), GUILayout.ExpandWidth(true), GUILayout.Width(100)),
                out SmokeScreenConfig.Instance.maximumActiveParticles);
            GUILayout.EndHorizontal();

            GUILayout.Label("activeParticles : " + SmokeScreenConfig.activeParticles);

            GUILayout.Space(10);

            GUILayout.Label("Open ModelMultiParticlePersistFX UI :");

            foreach (var mmFX in ModelMultiParticlePersistFX.List)
            {
                mmFX.showUI = GUILayout.Toggle(
                    mmFX.showUI,
                    mmFX.hostPart.name + " " + mmFX.effectName + " " + mmFX.instanceName);
            }

            GUILayout.EndVertical();

            GUI.DragWindow();
        }
    }
}
