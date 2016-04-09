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

using SmokeScreen;
using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

[EffectDefinition("MODEL_MULTI_PARTICLE_PERSIST")]
public class ModelMultiParticlePersistFX : EffectBehaviour
{
    #region Persistent fields

    [Persistent] public string modelName = string.Empty;

    [Persistent] public string transformName = string.Empty;

    [Persistent] public string shaderFileName = string.Empty;

    [Persistent] public string renderMode = "Billboard";

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

    private List<PersistentKSPParticleEmitter> persistentEmitters;

    private Shader shader;

    public string node_backup = string.Empty;

    private bool activated = true;

    private bool loaded = false;

    public bool showUI = false;

    private static readonly List<ModelMultiParticlePersistFX> list = new List<ModelMultiParticlePersistFX>();

    public float specialScale = 1;

    private float singleTimerEnd = 0;

    private float timeModuloDelta = 0;

    private string lastRenderMode = "";

    public static List<ModelMultiParticlePersistFX> List
    {
        get { return list; }
    }

    public bool overRideInputs = false;

    public readonly float[] inputs = new float[MultiInputCurve.inputsCount];

    public List<ModelMultiParticlePersistFX> Instances
    {
        get { return list; }
    }

    public ModelMultiParticlePersistFX()
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
    // vessel gameobject are removed without an OnDestroy call
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
                persistentEmitters[i].fixedEmit = true;
                persistentEmitters[i].pe.emit = false;
            }
        }
        else
        {
            for (int j = 0; j < persistentEmitters.Count; j++)
            {
                persistentEmitters[j].fixedEmit = false;
                persistentEmitters[j].pe.emit = false;
            }
        }
    }

    public void FixedUpdate()
    {
        //Print("FixedUpdate");
        if (persistentEmitters == null || hostPart == null || hostPart.rb == null)
        {
            return;
        }

        if (singleTimerEnd > 0)
        {
            if (Time.fixedTime <= singleTimerEnd)
            {
                OnEvent(1);
            }
            else
            {
                OnEvent(0);
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

        PersistentKSPParticleEmitter[] persistentKspParticleEmitters = persistentEmitters.ToArray();
        for (int i = 0; i < persistentKspParticleEmitters.Length; i++)
        {
            PersistentKSPParticleEmitter persistentKspParticleEmitter = persistentKspParticleEmitters[i];

            persistentKspParticleEmitter.EmitterOnUpdate(hostPart.rb.velocity + Krakensbane.GetFrameVelocity());
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
            PersistentKSPParticleEmitter pkpe = persistentEmitters[i];

            if (pkpe.go == null)
                continue;

            //pkpe.pe.useWorldSpace
            float finalScale = fixedScale * specialScale;

            float finalSizeClamp = sizeClamp + sizeClampCurve.Value(inputs);

            float sizePower = size.Value(inputs) * finalScale;
            pkpe.pe.minSize = Mathf.Min(pkpe.minSizeBase * sizePower, finalSizeClamp);
            pkpe.pe.maxSize = Mathf.Min(pkpe.maxSizeBase * sizePower, finalSizeClamp);

            float emissionPower = emission.Value(inputs) * emissionMult;
            pkpe.pe.minEmission = Mathf.FloorToInt(pkpe.minEmissionBase * emissionPower);
            pkpe.pe.maxEmission = Mathf.FloorToInt(pkpe.maxEmissionBase * emissionPower);

            float energyPower = energy.Value(inputs);
            pkpe.pe.minEnergy = pkpe.minEnergyBase * energyPower;
            pkpe.pe.maxEnergy = pkpe.maxEnergyBase * energyPower;

            float velocityPower = speed.Value(inputs) * finalScale;
            pkpe.pe.localVelocity = pkpe.localVelocityBase * velocityPower;
            pkpe.pe.worldVelocity = pkpe.worldVelocityBase * velocityPower;

            float forcePower = force.Value(inputs);
            pkpe.pe.force = pkpe.forceBase * forcePower;

            pkpe.pe.sizeGrow = grow.Value(inputs);

            float currentScale = scale.Value(inputs) * finalScale;
            pkpe.pe.shape1D = pkpe.scale1DBase * currentScale;
            pkpe.pe.shape2D = pkpe.scale2DBase * currentScale;
            pkpe.pe.shape3D = pkpe.scale3DBase * currentScale;

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

            pkpe.linearGrow = linGrow.Value(inputs);

            if (alpha.Value(inputs) != 1 || linAlphaDecay.Value(inputs) != 0 || logAlphaDecay.Value(inputs) != 0)
            {
                Color[] cols = new Color[5];

                for (int t = 0; t < 5; t++)
                {
                    float a =
                        Mathf.Clamp01(alpha.Value(inputs) *
                                      (1 - linAlphaDecay.Value(inputs) * (t / 4f) -
                                       Mathf.Log(logAlphaDecay.Value(inputs) * (t / 4f) + 1)));
                    cols[t] = new Color(a, a, a, a);
                }

                pkpe.pe.colorAnimation = cols;
                pkpe.pe.doesAnimateColor = true;
            }

            pkpe.go.transform.localPosition = localPosition
                                              + offsetDirection.normalized * offset.Value(inputs) * finalScale;

            pkpe.go.transform.localRotation = Quaternion.Euler(localRotation);


            if (renderMode != lastRenderMode)
            {
                // Bad code is bad
                try
                {
                    pkpe.pe.particleRenderMode = (ParticleRenderMode)Enum.Parse(typeof (ParticleRenderMode), renderMode);
                }
                catch (ArgumentException) { }
                lastRenderMode = renderMode;
            }
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

            persistentEmitters[i].pe.maxParticleSize = persistentEmitters[i].maxSizeBase * angle.Value(currentAngle)
                                                       * distance.Value(currentDist);
            persistentEmitters[i].pe.pr.maxParticleSize = persistentEmitters[i].pe.maxParticleSize;
        }
    }

    public override void OnInitialize()
    {
        //Print("Init");

        // Restore the Curve config from the node content backup
        // Done because I could not get the serialization of MultiInputCurve to work
        if (node_backup != string.Empty)
        {
            Restore();
        }

        // The shader loading require proper testing
        // Unity doc says that "Creating materials this way supports only simple shaders (fixed function ones).
        // If you need a surface shader, or vertex/pixel shaders, you'll need to create shader asset in the editor and use that."
        // But importing the same shader that the one used in the editor seems to work
        string filename = KSPUtil.ApplicationRootPath + "GameData/" + shaderFileName;
        if (shaderFileName != string.Empty && File.Exists(filename))
        {
            try
            {
                TextReader shaderFile = new StreamReader(filename);
                string shaderText = shaderFile.ReadToEnd();
                shader = new Material(shaderText).shader;
            }
            catch (Exception e)
            {
                Print("unable to load shader " + shaderFileName + " : " + e);
            }
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

        if (shader != null)
        {
            templateKspParticleEmitter.material.shader = shader;
        }

        if (persistentEmitters == null)
        {
            persistentEmitters = new List<PersistentKSPParticleEmitter>();
        }

        if (hostPart.Modules.Contains("ProceduralSRB"))
        {
            PartModule pm = hostPart.Modules["ProceduralSRB"];

            specialScale = pm.Fields.GetValue<float>("bellScale");
            Print("Found ProceduralSRB. Rescaling by " + specialScale.ToString("F3") + " final scale " + (fixedScale * specialScale).ToString("F3"));
        }

        for (int i = 0; i < transforms.Count; i++)
        {
            GameObject emitterGameObject = Instantiate(model) as GameObject;
            KSPParticleEmitter childKSPParticleEmitter = emitterGameObject.GetComponentInChildren<KSPParticleEmitter>();

            if (shader != null)
            {
                childKSPParticleEmitter.material.shader = shader;
                childKSPParticleEmitter.pr.material.shader = shader;
            }

            if (childKSPParticleEmitter != null)
            {
                PersistentKSPParticleEmitter pkpe = new PersistentKSPParticleEmitter(
                    emitterGameObject,
                    childKSPParticleEmitter,
                    templateKspParticleEmitter);

                try
                {
                    childKSPParticleEmitter.particleRenderMode =
                        (ParticleRenderMode)Enum.Parse(typeof (ParticleRenderMode), renderMode);
                }
                catch (ArgumentException)
                {
                    Print("ModelMultiParticleFXExt: " + renderMode + " is not a valid ParticleRenderMode");
                }

                persistentEmitters.Add(pkpe);

                DisableCollider(pkpe.go);

                emitterGameObject.transform.SetParent(transforms[i]);

                emitterGameObject.transform.localPosition = localPosition;
                emitterGameObject.transform.localRotation = Quaternion.Euler(localRotation);
            }
        }

        Destroy(templateKspParticleEmitter);

        list.Add(this);

        // 1.0 don't seems to properly do this for engines.
        OnEvent(0);

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

    // TODO : I learned to do proper serialization since then. I might want to do it instead of that mess
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

    public override void OnLoad(ConfigNode node)
    {
        //Print("OnLoad");
        Backup(node);

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
        sizeClampCurve.Load(node);
        randConeEmit.Load(node);
        vRandPosOffset.Load(node);
        vPosOffset.Load(node);
        xyForce.Load(node);
        zForce.Load(node);

        angle.Load("angle", node);
        distance.Load("distance", node);
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
        print("[SmokeScreen ModelMultiParticlePersistFX] " + s);
    }

    // TODO : move the whole UI stuff to a dedicated class - this is getting way to big

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
            GUILayout.Label("Shader: " + persistentEmitters[0].pe.pr.material.shader.name);
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
                persistentEmitters[i].pe.pe.ClearParticles();
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
        max = Mathf.Max(max, sizeClampCurve.maxInput[id]);
        max = Mathf.Max(max, randConeEmit.minInput[id]);
        max = Mathf.Max(max, vRandPosOffset.minInput[id]);
        max = Mathf.Max(max, vPosOffset.minInput[id]);
        max = Mathf.Max(max, xyForce.minInput[id]);
        max = Mathf.Max(max, zForce.minInput[id]);

        return max;
    }
}