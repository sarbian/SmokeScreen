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

using UnityEngine;

namespace SmokeScreen
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    internal class SmokeScreenUI : MonoBehaviour
    {
        private const string icon = "SmokeScreen/SmokeScreen";

        private readonly IButton button;

        private bool showUI = false;

        private Rect winPos = new Rect(450, 50, 400, 100);

        private const int winID = 512099;

        private SmokeScreenUI()
        {
            if (!ToolbarManager.ToolbarAvailable)
            {
                return;
            }

            button = ToolbarManager.Instance.add("SmokeScreen", "main");
            button.TexturePath = icon;
            button.ToolTip = "SmokeScreen";
            button.OnClick += e => { showUI = !showUI; };
        }

        internal void Update()
        {
            if (GameSettings.MODIFIER_KEY.GetKey() && Input.GetKeyDown(KeyCode.P))
                showUI = !showUI;
        }

        private void OnDestroy()
        {
            if (button != null)
            {
                button.Destroy();
            }
        }

        private void OnGUI()
        {
            if (showUI)
            {
                winPos = GUILayout.Window(winID, winPos, windowGUI, "SmokeScreen", GUILayout.MinWidth(300));
            }
        }

        private void windowGUI(int ID)
        {
            GUILayout.BeginVertical();

            SmokeScreenConfig.Instance.globalCollideDisable =
                GUILayout.Toggle(SmokeScreenConfig.Instance.globalCollideDisable, "Globally disable Collide");
            SmokeScreenConfig.Instance.globalPhysicalDisable =
                GUILayout.Toggle(SmokeScreenConfig.Instance.globalPhysicalDisable, "Globally disable Physical");

            GUILayout.Space(10);

            GUILayout.BeginHorizontal();
            GUILayout.Label("maximumActiveParticles", GUILayout.ExpandWidth(true));
            int.TryParse(
                GUILayout.TextField(
                    SmokeScreenConfig.Instance.maximumActiveParticles.ToString(),
                    GUILayout.ExpandWidth(true),
                    GUILayout.Width(100)),
                out SmokeScreenConfig.Instance.maximumActiveParticles);
            GUILayout.EndHorizontal();

            GUILayout.Label("activeParticles : " + SmokeScreenConfig.activeParticles);

            GUILayout.Space(10);

            GUILayout.Label("Open ModelMultiParticlePersistFX UI :");

            foreach (var mmFX in ModelMultiParticlePersistFX.List)
            {
                if (mmFX.hostPart != null)
                {
                    mmFX.showUI = GUILayout.Toggle(
                        mmFX.showUI,
                        mmFX.hostPart.name + " " + mmFX.effectName + " " + mmFX.instanceName);
                }
            }

            GUILayout.EndVertical();

            GUI.DragWindow();
        }
    }
}