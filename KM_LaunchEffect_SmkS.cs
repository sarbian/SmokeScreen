
/*
 * Copyright (c) 2019, Sébastien GAGGINI AKA Sarbian, France
 * All rights reserved.
 *
 * Original work by dtobi, used with his permission
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


using System.Collections.Generic;
using UnityEngine;
using Random = System.Random;

namespace SmokeScreen
{
    public class KM_LaunchEffect_SmkS : PartModule
    {

        [KSPField(isPersistant = false)]
        public string model = string.Empty;

        [KSPField(isPersistant = false)]
        public string parent = string.Empty;

        [KSPField(isPersistant = false)]
        public Vector3 position = Vector3d.zero;

        [KSPField(isPersistant = false)]
        public Vector3 rotation = Vector3d.zero;

        [KSPField(isPersistant = false)]
        public Vector3 scale = Vector3d.one;

        [KSPField(isPersistant = false)]
        public string effectName = "PreLaunchEffect";

        [KSPField(isPersistant = false)]
        public string effectLightName = "PreLaunchEffectLight";

        [KSPField(isPersistant = false)]
        public string transformName = "";

        [KSPField(isPersistant = false)]
        public bool checkBottomNode = true;

        [KSPField(isPersistant = false)]
        public bool debug = false;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "Show"),
            UI_Toggle(disabledText = "Off", enabledText = "Editor")]
        public bool editorPlacementOptionsActive = false;

        [KSPField(isPersistant = true)]
        public bool isRunning = false;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "Timer"),
            UI_FloatRange(minValue = 0f, maxValue = 20f, stepIncrement = 1f)]
        public float runningTime = 0.0f;

        private float startTime = 0f;
        private bool timerRunning = false;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "With"),
            UI_FloatRange(minValue = 0f, maxValue = 2f, stepIncrement = 0.1f)]
        public float width = 0f;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "Height"),
            UI_FloatRange(minValue = 0f, maxValue = 20f, stepIncrement = 1f)]
        public float height = 0;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = false, guiName = "offset")]
        public Vector3 offset = Vector3d.zero;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "X"),
            UI_FloatRange(minValue = -4f, maxValue = 4f, stepIncrement = 0.1f)]
        public float xOffset = 0;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "Y"),
            UI_FloatRange(minValue = -4f, maxValue = 4f, stepIncrement = 0.1f)]
        public float yOffset = 0;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "Z"),
            UI_FloatRange(minValue = -4f, maxValue = 4f, stepIncrement = 0.1f)]
        public float zOffset = 0;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "Num P"),
            UI_FloatRange(minValue = 10, maxValue = 200f, stepIncrement = 20f)]
        public float numP = 0;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = false, guiName = "speed")]
        public Vector3 speed = Vector3d.zero;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "speedX"),
            UI_FloatRange(minValue = -5f, maxValue = 5f, stepIncrement = 0.25f)]
        public float speedX = 0;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "speedY"),
            UI_FloatRange(minValue = -5f, maxValue = 5f, stepIncrement = 0.25f)]
        public float speedY = 0;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "speedZ"),
            UI_FloatRange(minValue = -5f, maxValue = 5f, stepIncrement = 0.25f)]
        public float speedZ = 0;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "Size"),
            UI_FloatRange(minValue = -0f, maxValue = 1.5f, stepIncrement = 0.05f)]
        public float size = 0f;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "Random"),
            UI_FloatRange(minValue = -0f, maxValue = 2f, stepIncrement = 0.1f)]
        public float rndVelocity = 0f;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "SoftOff"),
            UI_Toggle(disabledText = "Disabled", enabledText = "Enabled")]
        public bool softDecrease = false;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "Launch Effects"),
            UI_Toggle(disabledText = "Disabled", enabledText = "Enabled")]
        public bool isActive = true;

        protected static Random rnd = new Random();


        private int performanceLimiter = rnd.Next();
        private int performaceThreshold = 10;

        private Dictionary<int, KSPParticleEmitter> effectsList = new Dictionary<int, KSPParticleEmitter>();
        private Dictionary<int, Vector3> locationList = new Dictionary<int, Vector3>();
        private Dictionary<int, Light> lightList = new Dictionary<int, Light>();


        [KSPEvent(guiName = "Toggle", guiActive = true)]
        public void toggle()
        {
            setEffect(!isActive);
        }


        Vessel.Situations lastSituation;
        
        private GameObject effectModel;


        [KSPAction("Toggle")]
        public void toggleAG(KSPActionParam param)
        {
            setEffect(!isActive);
        }

        public override void OnStart(StartState state)
        {
            if (checkBottomNode)
            {
                foreach (var node in this.part.attachNodes)
                {
                    print("Att:" + node.id);
                    if (node.id == "bottom" && node.attachedPart != null)
                    {
                        print("Attached!");
                        isActive = false;
                    }
                }
            }


            // Loads the effectModel manually. Avoids the need for a MODEL node
            // But may add a effectModel more than once if more than 1 KM_LaunchEffect_SmkS
            // module is added with that uses the same effectModel. 
            if (model != string.Empty)
            {
                effectModel = GameDatabase.Instance.GetModel(model);
                if (effectModel == null)
                {
                    print("Cannot find effectModel " + model);
                    return;
                }
                effectModel.SetActive(true);
                Transform parentTransform = part.transform.Find(parent);
                //print("ParentTransform Null = " + (parentTransform==null));
                effectModel.transform.SetParent(parentTransform ?? part.transform);
                effectModel.transform.localPosition = position;
                //print("position = " + position.ToString("F2"));
                effectModel.transform.localScale = scale;
                //print("scale = " + scale.ToString("F2"));
                effectModel.transform.localRotation = Quaternion.Euler(rotation);
                //print("rotation = " + rotation.ToString("F2"));
            }


            if (xOffset != 0 || yOffset != 0 || zOffset != 0)
                offset = new Vector3(xOffset, yOffset, zOffset);

            if (speedX != 0 || speedY != 0 || speedZ != 0)
                speed = new Vector3(speedX, speedY, speedZ);

            registerEffects();


            if (state != StartState.Editor && Vessel.Situations.PRELAUNCH == vessel.situation)
            {
                OnPreLaunch();
            }

            if (!debug)
            {

                this.Fields["runningTime"].guiActiveEditor = false;
                this.Fields["width"].guiActiveEditor = false;
                this.Fields["height"].guiActiveEditor = false;
                this.Fields["xOffset"].guiActiveEditor = false;
                this.Fields["yOffset"].guiActiveEditor = false;
                this.Fields["zOffset"].guiActiveEditor = false;
                this.Fields["numP"].guiActiveEditor = false;
                this.Fields["speedX"].guiActiveEditor = false;
                this.Fields["speedY"].guiActiveEditor = false;
                this.Fields["speedZ"].guiActiveEditor = false;
                this.Fields["size"].guiActiveEditor = false;
                this.Fields["rndVelocity"].guiActiveEditor = false;
                this.Fields["softDecrease"].guiActiveEditor = false;
                if (Events["toggle"] != null) Events["toggle"].guiActiveEditor = false;
            }
        }

        void registerEffects()
        {
            var launchEffects = effectModel == null ?  this.part.GetComponentsInChildren<KSPParticleEmitter>() : effectModel.GetComponentsInChildren<KSPParticleEmitter>() ;
            foreach (KSPParticleEmitter launchEffect in launchEffects)
            {
                if (launchEffect != null)
                {
                    //print ("DBG Found Effect: " + launchEffect.name);
                    if (launchEffect.name == effectName)
                    {
                        effectsList.Add(launchEffect.GetInstanceID(), launchEffect);
                        if (transformName == "" || transformName == effectName)
                        {
                            //print ("DBG: LP");
                            locationList.Add(launchEffect.GetInstanceID(), launchEffect.transform.localPosition);
                        }
                        else
                        {
                            //print ("DBG: MB");
                            var partTransforms = this.part.GetComponentsInChildren<MonoBehaviour>();
                            foreach (MonoBehaviour partTransform in partTransforms)
                            {
                                if (transformName == partTransform.name)
                                {
                                    launchEffect.transform.localPosition = partTransform.transform.localPosition;
                                    locationList.Add(launchEffect.GetInstanceID(), partTransform.transform.localPosition);
                                }
                            }
                        }

                        launchEffect.shape2D = new Vector2(width, width);
                        if (rndVelocity != 0)
                            launchEffect.rndVelocity = new Vector3(rndVelocity, rndVelocity, rndVelocity);

                        if (height != 0)
                            launchEffect.maxEnergy = height;
                        if (height != 0)
                            launchEffect.minEnergy = height / 2;

                        if (size != 0)
                            launchEffect.minSize = size;
                        if (size != 0)
                            launchEffect.minSize = size / 2;

                        if (!offset.IsZero())
                            launchEffect.transform.Translate(offset, Space.Self);

                        if (!speed.IsZero())
                            launchEffect.localVelocity = speed;

                    }
                }
            }
            //print ("DBG: BeforeLights");
            var launchEffectsLights = effectModel == null ? this.part.GetComponentsInChildren<Light>() : effectModel.GetComponentsInChildren<Light>();
            foreach (Light launchEffectLight in launchEffectsLights)
            {
                if (launchEffectLight != null)
                {
                    // print ("Found Light: " + launchEffectLight.name);
                    if (launchEffectLight.name == effectLightName)
                    {
                        // print ("Added light");
                        lightList.Add(launchEffectLight.GetInstanceID(), launchEffectLight);
                    }
                }
            }
            //print ("DBG: AfterLights");
        }


        public virtual void OnPreLaunch() { }

        public virtual void OnLaunch() { }

        public virtual void OnTimer() { }

        protected void startTimer()
        {

            timerRunning = true;
            startTime = Time.fixedTime;
        }

        protected float remainingTime()
        {
            return !timerRunning ? 0 : startTime + runningTime - Time.fixedTime;
        }

        protected void checkTimer()
        {
            if (remainingTime() < 0)
            {
                startTime = 0;
                timerRunning = false;
                OnTimer();
            }
        }

        public override void OnUpdate()
        {
            if (HighLogic.LoadedSceneIsEditor)
            {
                updateEditor();
                return;
            }

            // we launched
            if (lastSituation == Vessel.Situations.PRELAUNCH && vessel.situation != Vessel.Situations.PRELAUNCH)
            {
                OnLaunch();
            }
            checkTimer();

            lastSituation = vessel.situation;
        }

        public override string GetInfo()
        {
            return "KM Launch Effect by dtobi";
        }

        private void updateEditor()
        {
            if (performanceLimiter++ % performaceThreshold == 0)
                setEffect(editorPlacementOptionsActive);
        }
        
        public void setEffect(bool state)
        {
            foreach (KeyValuePair<int, KSPParticleEmitter> pair in effectsList)
            {
                KSPParticleEmitter launchEffect = pair.Value;
                //print ("Found Effect: " + launchEffect.name+" state"+state);
                launchEffect.emit = state;
                if (state)
                {
                    if (numP != 0)
                        launchEffect.maxEmission = (int)(numP);
                    if (numP != 0)
                        launchEffect.minEmission = (int)(numP / 2);
                }
            }
            foreach (Light launchEffectLight in lightList.Values)
            {
                launchEffectLight.intensity = (state ? 1 : 0);
            }

        }
    }



    public class KM_PreLaunchEffect_SmkS : KM_LaunchEffect_SmkS
    {


        public override void OnLaunch()
        {
            if (runningTime > 0)
            {
                startTimer();
            }
            else
            {
                setEffect(false);
                Events["toggle"].guiActive = false;
            }
        }

        public override void OnPreLaunch()
        {
            if (isActive)
            {
                setEffect(true);
            }
            else
            {
                print("Effect " + effectName + " not active. Not starting.");
            }

        }

        public override void OnTimer()
        {
            setEffect(false);
            //print("Shutting off effects:" + effectName);
        }
    }

    public class KM_PostLaunchEffect_SmkS : KM_LaunchEffect_SmkS
    {
        private int performanceLimiterUpdate = 0;
        private int performanceThreshold = 15;

        private int originalNumP = 0;

        public override void OnPreLaunch()
        {

            if (isActive) setEffect(false);
        }

        public override void OnUpdate()
        {
            if (HighLogic.LoadedSceneIsEditor)
                return;

            if (softDecrease && performanceLimiterUpdate++ % performanceThreshold == 0)
            {
                // we launched
                if (vessel.situation != Vessel.Situations.PRELAUNCH)
                {
                    var remT = remainingTime();
                    if (remT <= 0)
                    {
                        numP = 0;
                        setEffect(false);
                        softDecrease = false;
                    }
                    else
                    {
                        numP = originalNumP * remT / runningTime;
                        setEffect(true);
                    }

                }
            }


            checkTimer();

            base.OnUpdate();
        }


        public override void OnLaunch()
        {
            performanceLimiterUpdate = rnd.Next();
            originalNumP = (int)numP;
            if (isActive) setEffect(true);
            startTimer();
            Events["toggle"].guiActive = false;
        }

        public override void OnTimer()
        {
            setEffect(false);
            softDecrease = false;
            print("Shutting off effects:" + effectName);
        }
    }
}
