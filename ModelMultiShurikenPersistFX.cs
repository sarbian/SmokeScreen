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

using SmokeScreen;
using System;
using System.Collections.Generic;
using UniLinq;
using UnityEngine;

[EffectDefinition("MODEL_MULTI_SHURIKEN_PERSIST")]
public class ModelMultiShurikenPersistFX : EffectBehaviour
{
    #region Persistent fields

    [Persistent] public string modelName = string.Empty;

    [Persistent] public string transformName = string.Empty;
    
    [Persistent] public string renderMode = "Billboard";

    [Persistent] public string layer = "TransparentFX";
    private int layerId = 1;

    [Persistent] public bool collide = false;

    [Persistent] public float collideRatio = 0.0f;

    [Persistent] public Vector3 localRotation = Vector3.zero;

    [Persistent] public Vector3 localPosition = Vector3.zero;

    [Persistent] public Vector3 offsetDirection = Vector3.forward;

    [Persistent] public float fixedScale = 1;

    [Persistent] public float emissionMult = 1;

    [Persistent] public float sizeClamp = 50;

    // Initial density of the particle seen as sphere of radius size of perfect
    // gas. We then assume (only true for ideally expanded exhaust) that the
    // expansion is isobaric (by mixing with the atmosphere) in order to compute
    // the density afterwards. Units (SI): kg / m^3.
    [Persistent] public double initialDensity = .6;

    // Whether to apply Archimedes' force, gravity and other things to the
    // particle.
    [Persistent] public bool physical = false;

    // How much the particles stick to objects they collide with.
    [Persistent] public double stickiness = 0.9;

    [Persistent] public double dragCoefficient = 0.1;

    // Current Time % timeModulo is used as the time input
    [Persistent] public float timeModulo = 10;

    // For how long the effect will be running after a single Emit()
    // time input is overridden to be the remaining time while it runs
    [Persistent] public float singleEmitTimer = 0;

    // The initial velocity of the particles will be offset by a random amount
    // lying in a disk perpendicular to the mean initial velocity whose radius
    // is randomOffsetMaxRadius. This is similar to Unity's 'Random Velocity'
    // Setting, except it will sample the offset from a (normal) disk rather
    // than from a cube. Units (SI): m/s.
    // TODO Sarbian : have the init auto fill this one
    [Persistent] public float randomInitalVelocityOffsetMaxRadius = 0.0f;

    // Enables particle declustering
    // This adds a vector to particle's position based on velocity, deltaTime, and which particle of the frame is it.
    // ⁙    ⁙    ⁙    ⁙    ⁙    ⁙    ⁙
    // ^      false
    // SPAWNED IN ONE FRAME
    // vvvvv  true
    // ···································
    [Persistent] public bool decluster = false;

    // Emits particles on LateUpdate, rather than FixedUpdate, if enabled
    //
    // Synchronizes particle emission with frame draws. That fixes the 'sliding away from origin' on lower FPS
    // (I think new particles effectively skip first physics pass before drawing)
    // Check here (Rightmost one is enabled): https://i.imgur.com/fTU2F7r.gif
    // 
    // Also, makes decluster more consistent. Frame draws are not synchronized with FixedUpdate.
    // Decluster relies on Time.deltaTime to calculate distance to last emmited particle.
    // On FixedUpdate, the Time.deltaTime is always 0.02, regardless of how much time actually passed from last frame draw.
    // On LateUpdate,  the Time.deltaTime is the actual time from last draw, so decluster can predict last particle's position a lot better
    [Persistent] public bool emitOnUpdate = false;
    private bool EmitOnUpdate => emitOnUpdate || SmokeScreenConfig.Instance.forceEmitOnUpdate || TimeWarp.WarpMode == TimeWarp.Modes.HIGH;

    [Persistent]
    public int particleCountLimit = 1000;

    #endregion Persistent fields

    public MultiInputCurve emission;

    public MultiInputCurve energy;

    public MultiInputCurve speed;

    public MultiInputCurve grow;

    public MultiInputCurve scale;

    public MultiInputCurve size;

    public MultiInputCurve offset;

    public MultiInputCurve force;

    public MultiInputCurve logGrowScale;

    // Logarithmic growth applied to to the particle.
    // The size at time t after emission will be approximately
    // (Log(logarithmicGrowth * t + 1) + 1) * initialSize, assuming grow = 0.
    public MultiInputCurve logGrow;

    public MultiInputCurve linGrow;

    public MultiInputCurve alpha;

    public MultiInputCurve logAlphaDecay;

    public MultiInputCurve linAlphaDecay;
    
    public MultiInputCurve saturationMult;

    public MultiInputCurve brightnessMult;

    public MultiInputCurve alphaMult;

    public MultiInputCurve initalVelocityOffsetMaxRadius;

    public MultiInputCurve randConeEmit;

    public MultiInputCurve vRandPosOffset;

    public MultiInputCurve vPosOffset;

    public MultiInputCurve xyForce;

    public MultiInputCurve zForce;
    
    public MultiInputCurve sizeClampCurve;

    // Those 2 curve are related to the angle and distance to cam
    public FXCurve angle = new FXCurve("angle", 1f);

    public FXCurve distance = new FXCurve("distance", 1f);

    private List<PersistentKSPShurikenEmitter> persistentEmitters;

    private int lastMaxActiveParticles = 0;

    public int MaxActiveParticles
    {
        get
        {
            int max = persistentEmitters.Max(emitter => emitter.pe.particleCount);
            lastMaxActiveParticles = Mathf.Max(max, lastMaxActiveParticles);
            return lastMaxActiveParticles;
        }
    }

    // Previous way of counting particle count relied on the particles being emitted/updated synchronously with each other.
    // With 'emitOnUpdate' changes, particles can be updated either in FixedUpdate, or LateUpdate.
    // Used to count all currently active particles, and to display current particle count of this effect in SmokeScreen UI.
    public int CurrentlyActiveParticles
    {
        get
        {
            int count = 0;
            for (int i = 0; i < persistentEmitters.Count; i++)
            {
                var emitter = persistentEmitters[i];
                if (emitter != null && emitter.pe != null)
                    count += emitter.pe.particleCount;
            }
            return count;
        }
    }

    public string node_backup = string.Empty;

    private bool activated = true;

    private bool loaded = false;

    public bool showUI = false;

    private static readonly List<ModelMultiShurikenPersistFX> list = new List<ModelMultiShurikenPersistFX>();

    public float specialScale = 1;

    private float singleTimerEnd = 0;

    private float timeModuloDelta = 0;

    public static List<ModelMultiShurikenPersistFX> List => list;

    public bool overRideInputs = false;

    public readonly float[] inputs = new float[MultiInputCurve.inputsCount];

    public ModelMultiShurikenPersistFX()
    {
        winID = baseWinID++;

        //Print("Constructor");
    }

    //~ModelMultiParticlePersistFX()
    //{
    //    print("DESTROY ALL HUMAN");
    //    list.Remove(this);
    //}

    // if Die() is called for a debris of vessel then all the
    // vessel gameObject are removed without an OnDestroy call
    // But an OnVesselDie message is send before, so we can
    // Catch in time
    public void OnVesselDie()
    {
        //Print("OnVesselDie");
        OnDestroy();
    }

    private void OnDestroy()
    {
        //Print("OnDestroy");
        if (persistentEmitters != null)
        {
            //Print("OnDestroy Detach");
            for (int i = 0; i < persistentEmitters.Count; i++)
            {
                persistentEmitters[i].Detach(0);
            }
        }
        list.Remove(this);
    }

    public override void OnEvent()
    {
        //Print("OnEvent");
        if (!activated || persistentEmitters == null)
        {
            return;
        }
        singleTimerEnd = singleEmitTimer + Time.fixedTime;
        timeModuloDelta = singleTimerEnd % timeModulo;

        // Old version Emitted 1 second of particle in one go
        // New version set power to 1 for singleEmitTimer seconds
        //UpdateEmitters(1);
        //for (int i = 0; i < persistentEmitters.Count; i++)
        //{
        //    persistentEmitters[i].pe.Emit();
        //}
    }

    public override void OnEvent(float power)
    {
        if (persistentEmitters == null)
        {
            return;
        }

        if ((overRideInputs || power > 0) && activated)
        {
            UpdateEmitters(power);
            for (int i = 0; i < persistentEmitters.Count; i++)
            {
                PersistentKSPShurikenEmitter pkse = persistentEmitters[i];
                pkse.emitting = true;
                if (pkse.pe != null)
                {
                    ParticleSystem.EmissionModule em = pkse.pe.emission;
                    em.enabled = false;
                }
            }
        }
        else
        {
            for (int j = 0; j < persistentEmitters.Count; j++)
            {
                PersistentKSPShurikenEmitter pkse = persistentEmitters[j];
                pkse.emitting = false;
                if (pkse.pe != null)
                {
                    ParticleSystem.EmissionModule em = pkse.pe.emission;
                    em.enabled = false;
                }
            }
        }
    }

    public override void OnEvent(float power, int transformIdx)
    {
        if (persistentEmitters == null || transformIdx >= persistentEmitters.Count)
        {
            return;
        }

        if (transformIdx == -1)
        {
            OnEvent(power);
            return;
        }
        
        PersistentKSPShurikenEmitter pkse = persistentEmitters[transformIdx];
        if ((overRideInputs || power > 0) && activated)
        {
            UpdateEmitters(power);

            pkse.emitting = true;
            if (pkse.pe != null)
            {
                ParticleSystem.EmissionModule em = pkse.pe.emission;
                em.enabled = false;
            }
        }
        else
        {
            pkse.emitting = false;
            if (pkse.pe != null)
            {
                ParticleSystem.EmissionModule em = pkse.pe.emission;
                em.enabled = false;
            }
        }
    }

    public void FixedUpdate()
    {
        //Print("FixedUpdate");
        if (persistentEmitters == null || hostPart == null || hostPart.Rigidbody == null)
        {
            return;
        }

        if (singleTimerEnd > 0)
        {
            if (Time.fixedTime <= singleTimerEnd)
            {
                OnEvent(1f);
            }
            else
            {
                OnEvent(0f);
                singleTimerEnd = 0;
            }
        }
        SmokeScreenConfig.UpdateParticlesCount();

        //RaycastHit vHit = new RaycastHit();
        //Ray vRay = Camera.main.ScreenPointToRay(Input.mousePosition);
        //if(Physics.Raycast(vRay, out vHit))
        //{
        //    RaycastHit vHit2 = new RaycastHit();
        //    if (Physics.Raycast(vHit.point + vHit.normal * 10, -vHit.normal, out vHit2))
        //        Debug.Log(vHit2.collider.name);
        //}

        for (int i = 0; i < persistentEmitters.Count; i++)
        {
            PersistentKSPShurikenEmitter emitter = persistentEmitters[i];
            // This is FixedUpdate, so don't emit here if particles should emit in LateUpdate
            if (!EmitOnUpdate)
            {
                emitter.EmitterOnUpdate(hostPart.Rigidbody.velocity + Krakensbane.GetFrameVelocity());
            }
        }
    }

    private void UpdateInputs(float power)
    {
        if (overRideInputs)
        {
            return;
        }

        float atmDensity = 1;
        float surfaceVelMach = 1;
        float partTemp = 1;
        float externalTemp = 1;

        float time;
        if (Time.fixedTime > singleTimerEnd)
        {
            // timeModuloDelta makes the transition between the two state smooth
            time = (Time.fixedTime - timeModuloDelta) % timeModulo;
        }
        else
        {
            time = singleTimerEnd - Time.fixedTime;
        }

        if (hostPart != null)
        {
            partTemp = (float)hostPart.temperature;
            atmDensity = (float)hostPart.atmDensity;
            surfaceVelMach = (float)hostPart.machNumber;

            if (hostPart.vessel != null)
            {
                externalTemp = (float)hostPart.vessel.externalTemperature;
            }
            else
            {
                externalTemp = (float)FlightGlobals.getExternalTemperature(hostPart.transform.position);
            }
        }

        inputs[(int)MultiInputCurve.Inputs.power] = power;
        inputs[(int)MultiInputCurve.Inputs.density] = (float)Math.Pow(atmDensity,SmokeScreenConfig.Instance.atmDensityExp);
        inputs[(int)MultiInputCurve.Inputs.mach] = surfaceVelMach;
        inputs[(int)MultiInputCurve.Inputs.parttemp] = partTemp;
        inputs[(int)MultiInputCurve.Inputs.externaltemp] = externalTemp;
        inputs[(int)MultiInputCurve.Inputs.time] = time;
    }

    public void UpdateEmitters(float power)
    {
        UpdateInputs(power);

        for (int i = 0; i < persistentEmitters.Count; i++)
        {
            PersistentKSPShurikenEmitter pkpe = persistentEmitters[i];

            if (pkpe.go == null)
                continue;
            
            float finalScale = fixedScale * specialScale;

            float finalSizeClamp = sizeClamp + sizeClampCurve.Value(inputs);

            float sizePower = size.Value(inputs) * finalScale;
            pkpe.minSize = Mathf.Min(pkpe.minSizeBase * sizePower, finalSizeClamp);
            pkpe.maxSize = Mathf.Min(pkpe.maxSizeBase * sizePower, finalSizeClamp);

            float emissionPower = emission.Value(inputs) * emissionMult;
            pkpe.minEmission = Mathf.FloorToInt(pkpe.minEmissionBase * emissionPower);
            pkpe.maxEmission = Mathf.FloorToInt(pkpe.maxEmissionBase * emissionPower);

            float energyPower = energy.Value(inputs);
            pkpe.minEnergy = pkpe.minEnergyBase * energyPower;
            pkpe.maxEnergy = pkpe.maxEnergyBase * energyPower;

            float velocityPower = speed.Value(inputs) * finalScale;
            pkpe.localVelocity = pkpe.localVelocityBase * velocityPower;
            pkpe.worldVelocity = pkpe.worldVelocityBase * velocityPower;

            float forcePower = force.Value(inputs);
            pkpe.force = pkpe.forceBase * forcePower;

            ParticleSystem.ForceOverLifetimeModule fol = pkpe.pe.forceOverLifetime;
            fol.enabled = pkpe.force.sqrMagnitude > 0 || pkpe.rndForce.sqrMagnitude > 0;
            fol.x = new ParticleSystem.MinMaxCurve(pkpe.forceBase.x, pkpe.force.x + pkpe.rndForce.x);
            fol.y = new ParticleSystem.MinMaxCurve(pkpe.forceBase.y, pkpe.force.y + pkpe.rndForce.y);
            fol.z = new ParticleSystem.MinMaxCurve(pkpe.forceBase.z, pkpe.force.z + pkpe.rndForce.z);

            pkpe.sizeGrow = grow.Value(inputs);
            
            float currentScale = scale.Value(inputs) * finalScale;
            pkpe.shape1D = pkpe.scale1DBase * currentScale;
            pkpe.shape2D = pkpe.scale2DBase * currentScale;
            pkpe.shape3D = pkpe.scale3DBase * currentScale;

            pkpe.sizeClamp = finalSizeClamp;
            pkpe.randomInitalVelocityOffsetMaxRadius = randomInitalVelocityOffsetMaxRadius + initalVelocityOffsetMaxRadius.Value(inputs);

            pkpe.randConeEmit = randConeEmit.Value(inputs);
            pkpe.xyForce = xyForce.Value(inputs);
            pkpe.zForce = zForce.Value(inputs);

            pkpe.vRandPosOffset = vRandPosOffset.Value(inputs);
            pkpe.vPosOffset = vPosOffset.Value(inputs);

            pkpe.physical = physical && !SmokeScreenConfig.Instance.globalPhysicalDisable;
            pkpe.initialDensity = initialDensity;
            pkpe.dragCoefficient = dragCoefficient;

            pkpe.collide = collide && !SmokeScreenConfig.Instance.globalCollideDisable;
            pkpe.stickiness = stickiness;
            pkpe.collideRatio = collideRatio;

            pkpe.logarithmicGrow = logGrow.Value(inputs);
            pkpe.logarithmicGrowScale = logGrowScale.Value(inputs);

            pkpe.decluster = decluster;

            pkpe.emitOnUpdate = EmitOnUpdate;

            pkpe.linearGrow = linGrow.Value(inputs);

            if (alpha.Value(inputs) != 1 || linAlphaDecay.Value(inputs) != 0 || logAlphaDecay.Value(inputs) != 0)
            {
                //Color[] cols = new Color[5];
                
                GradientColorKey[] colorKeys = new GradientColorKey[5];
                GradientAlphaKey[] alphaKeys = new GradientAlphaKey[5];
                
                for (int t = 0; t < 5; t++)
                {
                    float a =
                        Mathf.Clamp01(alpha.Value(inputs) *
                                      (1 - linAlphaDecay.Value(inputs) * (t / 4f) -
                                       Mathf.Log(logAlphaDecay.Value(inputs) * (t / 4f) + 1)));
                    colorKeys[t] = new GradientColorKey(Color.red, t * 0.25f);
                    alphaKeys[t] = new GradientAlphaKey(a, t * 0.25f);
                }

                ParticleSystem.ColorOverLifetimeModule col = pkpe.pe.colorOverLifetime;
                col.enabled = true;
                
                Gradient gradient = new Gradient();
                gradient.SetKeys(colorKeys, alphaKeys);

                col.color = new ParticleSystem.MinMaxGradient(gradient);
            }

            pkpe.saturationMult = saturationMult.Value(inputs);
            pkpe.brightnessMult = brightnessMult.Value(inputs);
            pkpe.alphaMult = alphaMult.Value(inputs);

            pkpe.go.transform.localPosition = localPosition
                                              + offset.Value(inputs) * finalScale * offsetDirection.normalized;

            pkpe.go.transform.localRotation = Quaternion.Euler(localRotation);
        }
    }

    public void Update()
    {
        //Print("Update");
        if (persistentEmitters == null || Camera.main == null)
        {
            return;
        }

        for (int i = 0; i < persistentEmitters.Count; i++)
        {
            if (persistentEmitters[i].go == null)
                continue;

            // using Camera.main will mess up anything multi cam but using current require adding a OnWillRenderObject() to the ksp particle emitter GameObject (? not tested)
            float currentAngle = Vector3.Angle(
                -Camera.main.transform.forward,
                persistentEmitters[i].go.transform.forward);
            float currentDist = (Camera.main.transform.position - persistentEmitters[i].go.transform.position).magnitude;

            persistentEmitters[i].pr.maxParticleSize = persistentEmitters[i].maxSizeBase * angle.Value(currentAngle) * distance.Value(currentDist);
        }
    }

    // First, I tried to emit particles on regular Update, but stuff was weird, and the plume still appeared out of sync with frame draws
    // According to https://docs.unity3d.com/Manual/ExecutionOrder.html
    // LateUpdate is the last thing that happens before frame draw. That makes it as synced to frame draws, as possible
    // LateUpdate is called after physics calculations too, so the newly emitted plume particles are right where they should be.
    public void LateUpdate ()
    {
        if (persistentEmitters == null || hostPart == null || hostPart.Rigidbody == null)
        {
            return;
        }

        // I think it's important to call this even though it doesn't count active particles
        // because it calculates how many particles should be removed on next emit pass.
        SmokeScreenConfig.UpdateParticlesCount ();

        for (int i = 0; i < persistentEmitters.Count; i++)
        {
            PersistentKSPShurikenEmitter emitter = persistentEmitters[i];
            if (EmitOnUpdate)
            {
                emitter.EmitterOnUpdate(hostPart.Rigidbody.velocity + Krakensbane.GetFrameVelocity());
            }
        }
    }

    public override void OnInitialize()
    {
        //Print("OnInitialize");

        // Ship spawned somehow don't call OnLoad...
        if (node_backup != string.Empty)
        {
            Restore();
        }
        
        List<Transform> transforms = new List<Transform>(hostPart.FindModelTransforms(transformName));
        if (transforms.Count == 0)
        {
            Print("Cannot find transform " + transformName);
            return;
        }
        GameObject model = GameDatabase.Instance.GetModel(modelName);
        if (model == null)
        {
            Print("Cannot find model " + modelName);
            return;
        }
        model.SetActive(true);
        KSPParticleEmitter templateKspParticleEmitter = model.GetComponentInChildren<KSPParticleEmitter>();

        if (templateKspParticleEmitter == null)
        {
            Print("Cannot find particle emitter on " + modelName);
            Destroy(model);
            return;
        }
        
        if (persistentEmitters == null)
        {
            persistentEmitters = new List<PersistentKSPShurikenEmitter>();
        }

        if (hostPart.Modules.Contains("ProceduralSRB"))
        {
            PartModule pm = hostPart.Modules["ProceduralSRB"];

            specialScale = pm.Fields.GetValue<float>("bellScale");
            Print("Found ProceduralSRB. Rescaling by " + specialScale.ToString("F3") + " final scale " + (fixedScale * specialScale).ToString("F3"));
        }
        
        if (hostPart.Modules.Contains("TweakScale"))
        {
            PartModule pm = hostPart.Modules["TweakScale"];
            float tweakScale = pm.Fields.GetValue<float>("currentScale");
            float defaultScale = pm.Fields.GetValue<float>("defaultScale");
            specialScale = tweakScale / (defaultScale <= 0 ? 1 : defaultScale);
            Print("Found TweakScale. Rescaling by " + specialScale.ToString("F3") + " final scale " + (fixedScale * specialScale).ToString("F3"));
        }

        for (int i = 0; i < transforms.Count; i++)
        {
            GameObject emitterGameObject = i==0 ? model : Instantiate(model);
            KSPParticleEmitter childKSPParticleEmitter = emitterGameObject.GetComponentInChildren<KSPParticleEmitter>();

            if (childKSPParticleEmitter != null)
            {
                ParticleSystem particleSystem = childKSPParticleEmitter.gameObject.GetComponent<ParticleSystem>();
                ParticleSystemRenderer particleSystemRenderer = childKSPParticleEmitter.gameObject.GetComponent<ParticleSystemRenderer>();

                PersistentKSPShurikenEmitter pkpe = new PersistentKSPShurikenEmitter(
                    emitterGameObject,
                    particleSystem,
                    particleSystemRenderer,
                    childKSPParticleEmitter);


                // We do the emission and animation ourselves so we disable the KSP code
                childKSPParticleEmitter.emit = false;
                childKSPParticleEmitter.SetupProperties();
                childKSPParticleEmitter.enabled = false;

                ParticleSystem.MainModule main = particleSystem.main;
                main.maxParticles = particleCountLimit;
                
                //particleSystemRenderer.alignment = ParticleSystemRenderSpace.View;

                pkpe.doesAnimateColor = childKSPParticleEmitter.doesAnimateColor;

                if (childKSPParticleEmitter.doesAnimateColor)
                {
                    ParticleSystem.ColorOverLifetimeModule col = particleSystem.colorOverLifetime;

                    // This one is annoying. The old particle system animate the color over the whole % life of the particle (0 - 1)
                    // The new one animate it over time. So converted system may not reach the full value
                    // The color is manually set in the update of PersistentKSPShurikenEmitter
                    col.enabled = false;

                    Color[] colors = childKSPParticleEmitter.colorAnimation;
                    
                    pkpe.colors = new Color[colors.Length];
                    Array.Copy(colors, pkpe.colors, colors.Length);
                }

                //try
                //{
                //    particleSystemRenderer.renderMode =
                //        (ParticleSystemRenderMode)Enum.Parse(typeof (ParticleSystemRenderMode), renderMode);
                //}
                //catch (ArgumentException)
                //{
                //    Print("ModelMultiParticleFXExt: " + renderMode + " is not a valid ParticleSystemRenderMode");
                //}

                persistentEmitters.Add(pkpe);

                DisableCollider(pkpe.go);

                emitterGameObject.transform.SetParent(transforms[i], false);

                emitterGameObject.transform.localPosition = localPosition;
                emitterGameObject.transform.localRotation = Quaternion.Euler(localRotation);
                emitterGameObject.SetLayerRecursive(layerId);
            }
        }

        list.Add(this);

        // 1.0 don't seems to properly do this for engines.
        //OnEvent(0f);
    }

    private static void DisableCollider(GameObject go)
    {
        var collider = go.GetComponent<Collider>();
        if (collider != null)
        {
            //Print("Found one collider and disabled it");
            collider.enabled = false;
        }

        for (int i = 0; i < go.transform.childCount; i++)
        {
            if (go.transform.GetChild(i) != null)
            {
                DisableCollider(go.transform.GetChild(i).gameObject);
            }
        }
    }

    public void Backup(ConfigNode node)
    {
        node_backup = SmokeScreenUtil.WriteRootNode(node);
        //print("Backup node_backup is\n " + node_backup.Replace(Environment.NewLine, Environment.NewLine + "ModelMultiParticlePersistFX "));
    }

    public void Restore()
    {
        //print("Restore node_backup is\n " + node_backup.Replace(Environment.NewLine, Environment.NewLine + "ModelMultiParticlePersistFX "));
        ConfigNode node = ConfigNode.Parse(node_backup);
        OnLoad(node);
    }

    private ConfigNode GetEffectConfig()
    {
        ConfigNode partConfig = PartLoader.getPartInfoByName(hostPart.protoPartSnapshot.partName).partConfig;
        ConfigNode effectsNode = partConfig?.GetNode("EFFECTS");
        if (effectsNode == null) return null;
        ConfigNode[] eNodes = effectsNode.GetNodes(effectName);
        foreach (ConfigNode eNode in eNodes)
        {
            ConfigNode[] mmspNodes = eNode.GetNodes("MODEL_MULTI_SHURIKEN_PERSIST");
            foreach (ConfigNode mmspNode in mmspNodes)
                if (mmspNode.GetValue("name") == instanceName)
                    return mmspNode;
        }
        return null;
    }

    public override void OnLoad(ConfigNode node)
    {
        //print("OnLoad");

        // Use the actual part cfg instead of what KSP provides because
        // the node provided for the root part after a load/scene change is empty
        if (HighLogic.LoadedScene != GameScenes.LOADING && !node.HasNode())
        {
            node = GetEffectConfig();

            if (node == null)
            {
                Print("Unable to find the effect config for" +
                      " part " + hostPart.protoPartSnapshot.partName +
                      " effectName " + effectName +
                      " instanceName " + instanceName);
                return;
            }
        }

        // Backup the config because this effect config goes beyond the 7 deep serialization restriction and we get a node without the InputCurves
        // A cheap hack but fixing this takes way too much compile/reloading...

        Backup(node);

        //print("OnLoad :\n" + node);

        emission = new MultiInputCurve("emission");
        energy = new MultiInputCurve("energy");
        speed = new MultiInputCurve("speed");
        grow = new MultiInputCurve("grow", true);
        scale = new MultiInputCurve("scale");
        size = new MultiInputCurve("size");
        offset = new MultiInputCurve("offset", true);
        force = new MultiInputCurve("force", true);
        logGrow = new MultiInputCurve("logGrow", true);
        linGrow = new MultiInputCurve("linGrow", true);
        logGrowScale = new MultiInputCurve("logGrowScale");
        alpha = new MultiInputCurve("alpha");
        linAlphaDecay = new MultiInputCurve("linAlphaDecay", true);
        logAlphaDecay = new MultiInputCurve("logAlphaDecay", true);
        saturationMult = new MultiInputCurve("saturationMult");
        brightnessMult = new MultiInputCurve("brightnessMult");
        alphaMult = new MultiInputCurve("alphaMult");
        initalVelocityOffsetMaxRadius = new MultiInputCurve("initalVelocityOffsetMaxRadius", true);
        sizeClampCurve = new MultiInputCurve("sizeClamp", true);
        randConeEmit = new MultiInputCurve("randConeEmit", true);
        vRandPosOffset = new MultiInputCurve("vRandPosOffset", true);
        vPosOffset = new MultiInputCurve("vPosOffset", true);
        xyForce = new MultiInputCurve("xyForce", false);
        zForce = new MultiInputCurve("zForce", false);

        ConfigNode.LoadObjectFromConfig(this, node);
        emission.Load(node);
        energy.Load(node);
        speed.Load(node);
        grow.Load(node);
        scale.Load(node);
        size.Load(node);
        offset.Load(node);
        force.Load(node);
        logGrowScale.Load(node);
        logGrow.Load(node);
        linGrow.Load(node);
        alpha.Load(node);
        linAlphaDecay.Load(node);
        logAlphaDecay.Load(node);
        initalVelocityOffsetMaxRadius.Load(node);
        saturationMult.Load(node);
        brightnessMult.Load(node);
        alphaMult.Load(node);
        sizeClampCurve.Load(node);
        randConeEmit.Load(node);
        vRandPosOffset.Load(node);
        vPosOffset.Load(node);
        xyForce.Load(node);
        zForce.Load(node);

        angle.Load("angle", node);
        distance.Load("distance", node);
        layerId = LayerMask.NameToLayer(layer);

        loaded = true;
    }

    public override void OnSave(ConfigNode node)
    {
        if (!loaded)
        {
            //Print("OnSave called before any OnLoad");
            return;
        }

        // All those ugly null check are most likely superfluous after the loaded check
        // but I'll keep it until I m sure don't get NRE new report

        //Print("OnSave");
        ConfigNode.CreateConfigFromObject(this, node);
        if (emission != null)
        {
            emission.Save(node);
        }
        else
        {
            Print("OnSave emission is null");
        }
        if (energy != null)
        {
            energy.Save(node);
        }
        else
        {
            Print("OnSave energy is null");
        }
        if (speed != null)
        {
            speed.Save(node);
        }
        else
        {
            Print("OnSave speed is null");
        }
        if (grow != null)
        {
            grow.Save(node);
        }
        else
        {
            Print("OnSave grow is null");
        }
        if (scale != null)
        {
            scale.Save(node);
        }
        else
        {
            Print("OnSave scale is null");
        }
        if (size != null)
        {
            size.Save(node);
        }
        else
        {
            Print("OnSave size is null");
        }
        if (offset != null)
        {
            offset.Save(node);
        }
        else
        {
            Print("OnSave offset is null");
        }
        if (force != null)
        {
            force.Save(node);
        }
        else
        {
            Print("OnSave force is null");
        }
        if (logGrow != null)
        {
            logGrow.Save(node);
        }
        else
        {
            Print("OnSave logGrow is null");
        }
        if (linGrow != null)
        {
            linGrow.Save(node);
        }
        else
        {
            Print("OnSave linGrow is null");
        }
        if (logGrowScale != null)
        {
            logGrowScale.Save(node);
        }
        else
        {
            Print("OnSave logGrowScale is null");
        }
        if (alpha != null)
        {
            alpha.Save(node);
        }
        else
        {
            Print("OnSave alpha is null");
        }
        if (linAlphaDecay != null)
        {
            linAlphaDecay.Save(node);
        }
        else
        {
            Print("OnSave linAlphaDecay is null");
        }
        if (logAlphaDecay != null)
        {
            logAlphaDecay.Save(node);
        }
        else
        {
            Print("OnSave logAlphaDecay is null");
        }
        if (initalVelocityOffsetMaxRadius != null)
        {
            initalVelocityOffsetMaxRadius.Save(node);
        }
        else
        {
            Print("OnSave initalVelocityOffsetMaxRadius is null");
        }

        if (saturationMult != null)
        {
            saturationMult.Save(node);
        }
        else
        {
            Print("OnSave saturationMult is null");
        }
        if (brightnessMult != null)
        {
            brightnessMult.Save(node);
        }
        else
        {
            Print("OnSave brightnessMult is null");
        }

        if (alphaMult != null)
        {
            alphaMult.Save(node);
        }
        else
        {
            Print("OnSave alphaMult is null");
        }


        if (sizeClampCurve != null)
        {
            sizeClampCurve.Save(node);
        }
        else
        {
            Print("OnSave sizeClampCurve is null");
        }

        if (randConeEmit != null)
        {
            randConeEmit.Save(node);
        }
        else
        {
            Print("OnSave randConeEmit is null");
        }

        if (vRandPosOffset != null)
        {
            vRandPosOffset.Save(node);
        }
        else
        {
            Print("OnSave vRandPosOffset is null");
        }

        if (vPosOffset != null)
        {
            vPosOffset.Save(node);
        }
        else
        {
            Print("OnSave vPosOffset is null");
        }

        if (xyForce != null)
        {
            xyForce.Save(node);
        }
        else
        {
            Print("OnSave xyForce is null");
        }

        if (zForce != null)
        {
            zForce.Save(node);
        }
        else
        {
            Print("OnSave zForce is null");
        }

        if (angle != null)
        {
            angle.Save(node);
        }
        else
        {
            Print("OnSave angle is null");
        }
        if (distance != null)
        {
            distance.Save(node);
        }
        else
        {
            Print("OnSave distance is null");
        }
    }

    private static void Print(String s)
    {
        print("[SmokeScreen ModelMultiShurikenPersistFX] " + s);
    }

    // TODO : move the whole UI stuff to a dedicated class - this is getting way too big

    private Rect winPos = new Rect(50, 50, 400, 100);

    private static int baseWinID = 512100;

    private static int winID = baseWinID;

    private string nodeText = "";

    private bool nodeEdit = false;

    private Vector2 scrollPosition = new Vector2();

    private void OnGUI()
    {
        if (!HighLogic.LoadedSceneIsFlight)
        {
            return;
        }
        if (showUI && hostPart != null)
        {
            winPos = GUILayout.Window(
                winID,
                winPos,
                windowGUI,
                hostPart.name + " " + effectName + " " + instanceName,
                GUILayout.MinWidth(300));
        }
    }

    private void windowGUI(int ID)
    {
        GUILayout.BeginVertical();

        activated = GUILayout.Toggle(activated, "Active");

        GUILayout.Space(10);

        overRideInputs = GUILayout.Toggle(overRideInputs, "Manual Inputs");

        GUIInput((int)MultiInputCurve.Inputs.power, "Power");
        GUIInput((int)MultiInputCurve.Inputs.density, "Atmo Density");
        GUIInput((int)MultiInputCurve.Inputs.mach, "Mach Speed");
        GUIInput((int)MultiInputCurve.Inputs.parttemp, "Part Temperature");
        GUIInput((int)MultiInputCurve.Inputs.externaltemp, "External Temperature");

        if (persistentEmitters.Count > 0)
        {
            GUILayout.Label("Shader: " + persistentEmitters[0].pr.material.shader.name);
        }

        GUILayout.Space(10);

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Single Emit Timer"))
        {
            OnEvent();
        }

        if (GUILayout.Button("Clear Particles"))
        {
            for (int i = 0; i < persistentEmitters.Count; i++)
            {
                persistentEmitters[i].pe.Clear(true);
            }
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(10);

        nodeEdit = GUILayout.Toggle(nodeEdit, "Open Config Editor");

        if (nodeEdit)
        {
            GUILayout.BeginHorizontal();

            // Set the node with what was in the .cfg
            if (GUILayout.Button("Import"))
            {
                //nodeText = SmokeScreenUtil.WriteRootNode(GetEffectConfig());
                nodeText = string.Copy(node_backup);
                //print("Displaying node \n " + nodeText.Replace("\n", "\n" + "ModelMultiParticlePersistFX "));
            }

            // Rebuild the text from the active config
            if (GUILayout.Button("Rebuild"))
            {
                ConfigNode node = new ConfigNode();
                OnSave(node);
                nodeText = SmokeScreenUtil.WriteRootNode(node);
            }

            // Apply the text
            if (GUILayout.Button("Apply"))
            {
                ConfigNode node = ConfigNode.Parse(nodeText);
                OnLoad(node);
            }

            GUILayout.EndHorizontal();

            scrollPosition = GUILayout.BeginScrollView(scrollPosition, false, true, GUILayout.MinHeight(300));

            nodeText = GUILayout.TextArea(nodeText, GUILayout.ExpandWidth(true), GUILayout.ExpandHeight(true));
            GUILayout.EndScrollView();
        }

        GUILayout.EndVertical();

        GUI.DragWindow();
    }

    private readonly bool[] boxInput = new bool[MultiInputCurve.inputsCount];

    private void GUIInput(int id, string text)
    {
        float min = minInput(id);
        float max = maxInput(id);

        GUILayout.Label(
            text + " Val=" + inputs[id].ToString("F4") + " Min=" + min.ToString("F4") + " Max=" + max.ToString("F4"));

        if (overRideInputs)
        {
            GUILayout.BeginHorizontal();
            boxInput[id] = GUILayout.Toggle(boxInput[id], "", GUILayout.ExpandWidth(false));

            if (boxInput[id])
            {
                float.TryParse(
                    GUILayout.TextField(inputs[id].ToString("F4"), GUILayout.ExpandWidth(true), GUILayout.Width(100)),
                    out inputs[id]);
            }
            else
            {
                inputs[id] = GUILayout.HorizontalSlider(
                    inputs[id],
                    minInput(id),
                    maxInput(id),
                    GUILayout.ExpandWidth(true));
            }

            GUILayout.EndHorizontal();
        }
    }

    private float minInput(int id)
    {
        float min = emission.minInput[id];
        min = Mathf.Min(min, energy.minInput[id]);
        min = Mathf.Min(min, speed.minInput[id]);
        min = Mathf.Min(min, grow.minInput[id]);
        min = Mathf.Min(min, scale.minInput[id]);
        min = Mathf.Min(min, size.minInput[id]);
        min = Mathf.Min(min, offset.minInput[id]);
        min = Mathf.Min(min, force.minInput[id]);
        min = Mathf.Min(min, logGrow.minInput[id]);
        min = Mathf.Min(min, linGrow.minInput[id]);
        min = Mathf.Min(min, logGrowScale.minInput[id]);

        min = Mathf.Min(min, alpha.minInput[id]);
        min = Mathf.Min(min, linAlphaDecay.minInput[id]);
        min = Mathf.Min(min, logAlphaDecay.minInput[id]);
        min = Mathf.Min(min, initalVelocityOffsetMaxRadius.minInput[id]);
        min = Mathf.Min(min, saturationMult.minInput[id]);
        min = Mathf.Min(min, brightnessMult.minInput[id]);
        min = Mathf.Min(min, alphaMult.minInput[id]);
        min = Mathf.Min(min, sizeClampCurve.minInput[id]);
        min = Mathf.Min(min, randConeEmit.minInput[id]);
        min = Mathf.Min(min, vRandPosOffset.minInput[id]);
        min = Mathf.Min(min, vPosOffset.minInput[id]);
        min = Mathf.Min(min, xyForce.minInput[id]);
        min = Mathf.Min(min, zForce.minInput[id]);

        return min;
    }

    private float maxInput(int id)
    {
        float max = emission.maxInput[id];
        max = Mathf.Max(max, energy.maxInput[id]);
        max = Mathf.Max(max, speed.maxInput[id]);
        max = Mathf.Max(max, grow.maxInput[id]);
        max = Mathf.Max(max, scale.maxInput[id]);
        max = Mathf.Max(max, size.maxInput[id]);
        max = Mathf.Max(max, offset.maxInput[id]);
        max = Mathf.Max(max, force.maxInput[id]);
        max = Mathf.Max(max, logGrow.maxInput[id]);
        max = Mathf.Max(max, linGrow.maxInput[id]);
        max = Mathf.Max(max, logGrowScale.maxInput[id]);

        max = Mathf.Max(max, alpha.maxInput[id]);
        max = Mathf.Max(max, linAlphaDecay.maxInput[id]);
        max = Mathf.Max(max, logAlphaDecay.maxInput[id]);
        max = Mathf.Max(max, initalVelocityOffsetMaxRadius.maxInput[id]);
        max = Mathf.Max(max, saturationMult.maxInput[id]);
        max = Mathf.Max(max, brightnessMult.maxInput[id]);
        max = Mathf.Max(max, alphaMult.maxInput[id]);
        max = Mathf.Max(max, sizeClampCurve.maxInput[id]);
        max = Mathf.Max(max, randConeEmit.minInput[id]);
        max = Mathf.Max(max, vRandPosOffset.minInput[id]);
        max = Mathf.Max(max, vPosOffset.minInput[id]);
        max = Mathf.Max(max, xyForce.minInput[id]);
        max = Mathf.Max(max, zForce.minInput[id]);

        return max;
    }
}
