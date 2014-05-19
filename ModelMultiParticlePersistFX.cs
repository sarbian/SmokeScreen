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

using System;
using System.Collections.Generic;
using SmokeScreen;
using UnityEngine;

[EffectDefinition("MODEL_MULTI_PARTICLE_PERSIST")]
public class ModelMultiParticlePersistFX : EffectBehaviour
{
    #region Persistent fields

    [Persistent]
    public string modelName = string.Empty;

    [Persistent]
    public string transformName = string.Empty;

    [Persistent]
    public string shaderFileName = string.Empty;

    [Persistent]
    public string renderMode = "Billboard";

    [Persistent]
    public bool collide = false;

    [Persistent]
    public float collideRatio = 0.0f;

    [Persistent]
    public Vector3 localRotation = Vector3.zero;

    [Persistent]
    public Vector3 localPosition = Vector3.zero;

    [Persistent]
    public Vector3 offsetDirection = Vector3.forward;

    [Persistent]
    public float fixedScale = 1;

    [Persistent]
    public float sizeClamp = 50;

    // Initial density of the particle seen as sphere of radius size of perfect 
    // gas. We then assume (only true for ideally expanded exhaust) that the 
    // expansion is isobaric (by mixing with the atmosphere) in order to compute
    // the density afterwards. Units (SI): kg / m^3.
    [Persistent]
    public double initialDensity = .6;

    // Whether to apply Archimedes' force, gravity and other things to the 
    // particle.
    [Persistent]
    public bool physical = false;

    // How much the particles stick to objects they collide with.
    [Persistent]
    public double stickiness = 0.9;

    [Persistent]
    public double dragCoefficient = 0.1;

    /// <summary>
    /// Whether to nudge particles in order to alleviate the dotted smoke effect.
    /// Set this to true (default) when using 'Simulate World Space' in Unity,
    /// false otherwise.
    /// </summary>
    //[Persistent]
    public bool fixedEmissions = true;

    // The initial velocity of the particles will be offset by a random amount
    // lying in a disk perpendicular to the mean initial velocity whose radius
    // is randomOffsetMaxRadius. This is similar to Unity's 'Random Velocity'
    // Setting, except it will sample the offset from a (normal) disk rather
    // than from a cube. Units (SI): m/s.
    // TODO Sarbian : have the init auto fill this one
    [Persistent]
    public float randomInitalVelocityOffsetMaxRadius = 0.0f;

    #endregion Persistent fields

    public MultiInputCurve emission;

    public MultiInputCurve energy;

    public MultiInputCurve speed;

    public MultiInputCurve grow;

    public MultiInputCurve scale;

    public MultiInputCurve size;

    public MultiInputCurve offset;

    public MultiInputCurve force;

    // Logarithmic growth applied to to the particle.
    // The size at time t after emission will be approximately
    // (Log(logarithmicGrowth * t + 1) + 1) * initialSize, assuming grow = 0.
    public MultiInputCurve logGrow;

    private float logarithmicGrow;

    // Those 2 curve are related to the angle and distance to cam
    public FXCurve angle = new FXCurve("angle", 1f);

    public FXCurve distance = new FXCurve("distance", 1f);

    private List<PersistentKSPParticleEmitter> persistentEmitters;

    // The time between Update()s (in seconds).
    private float variableDeltaTime;

    private Shader shader;

    public string node_backup = string.Empty;

    private bool activated = true;

    public bool showUI = false;

    private static readonly List<ModelMultiParticlePersistFX> list = new List<ModelMultiParticlePersistFX>();

    public static List<ModelMultiParticlePersistFX> List
    {
        get
        {
            return list;
        }
    }

    public bool overRideInputs = false;

    public readonly float[] inputs = new float[MultiInputCurve.inputsCount];

    public List<ModelMultiParticlePersistFX> Instances
    {
        get
        {
            return list;
        }
    }

    public ModelMultiParticlePersistFX()
    {
        winID = baseWinID++;
    }

    //~ModelMultiParticlePersistFX()
    //{
    //    print("DESTROY ALL HUMAN");
    //    list.Remove(this);
    //}

    private void OnDestroy()
    {
        if (persistentEmitters != null)
        {
            for (int i = 0; i < persistentEmitters.Count; i++)
            {
                persistentEmitters[i].Detach(0);
            }
        }
        list.Remove(this);
    }

    public override void OnEvent()
    {
        if (!activated || persistentEmitters == null)
        {
            return;
        }

        UpdateEmitters(1);
        for (int i = 0; i < persistentEmitters.Count; i++)
        {
            persistentEmitters[i].pe.Emit();
        }
    }

    public override void OnEvent(float power)
    {
        if (persistentEmitters == null)
        {
            return;
        }

        if (power > 0f && activated)
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

    private bool addedLaunchPadCollider = false;

    public static uint physicsPass = 4;

    public static uint activePhysicsPass = 0;

    public void FixedUpdate()
    {
        if (persistentEmitters == null)
        {
            return;
        }

        SmokeScreenConfig.UpdateParticlesCount();

        RaycastHit hit = new RaycastHit();

        GameObject pad = GameObject.Find("ksp_pad_launchPad");

        //RaycastHit vHit = new RaycastHit();
        //Ray vRay = Camera.main.ScreenPointToRay(Input.mousePosition);
        //if(Physics.Raycast(vRay, out vHit))
        //{
        //    RaycastHit vHit2 = new RaycastHit();
        //    if (Physics.Raycast(vHit.point + vHit.normal * 10, -vHit.normal, out vHit2))
        //        Debug.Log(vHit2.collider.name);
        //}

        // "Default", "TransparentFX", "Local Scenery", "Ignore Raycast"
        int mask = (1 << LayerMask.NameToLayer("Default")) | (1 << LayerMask.NameToLayer("Local Scenery"));

        for (int i = 0; i < persistentEmitters.Count; i++)
        {
            // Emit particles on fixedUpdate rather than Update so that we know which particles
            // were just created and should be nudged, should not be collided, etc.
            if (persistentEmitters[i].fixedEmit)
            {
                // Number of particles to emit:
                double averageEmittedParticles = UnityEngine.Random.Range(
                    persistentEmitters[i].pe.minEmission,
                    persistentEmitters[i].pe.maxEmission) * TimeWarp.fixedDeltaTime;
                int emittedParticles = (int)Math.Floor(averageEmittedParticles)
                                       + (UnityEngine.Random.value
                                          < averageEmittedParticles - Math.Floor(averageEmittedParticles)
                                              ? 1
                                              : 0);
                for (int k = 0; k < emittedParticles; ++k)
                {
                    persistentEmitters[i].pe.EmitParticle();
                }
            }

            // This line (and the one that does the oposite at the end) is actally the slowest part of the whole function 
            Particle[] particles = persistentEmitters[i].pe.pe.particles;

            double averageSize = 0.5 * (persistentEmitters[i].pe.minSize + persistentEmitters[i].pe.maxSize);

            for (int j = 0; j < particles.Length; j++)
            {
                // Check if we need to cull the number of particles
                if (SmokeScreenConfig.particleDecimate != 0 && particles.Length > SmokeScreenConfig.decimateFloor)
                {
                    SmokeScreenConfig.particleCounter++;
                    if ((SmokeScreenConfig.particleDecimate > 0
                         && (SmokeScreenConfig.particleCounter % SmokeScreenConfig.particleDecimate) == 0)
                        || (SmokeScreenConfig.particleDecimate < 0
                            && (SmokeScreenConfig.particleCounter % SmokeScreenConfig.particleDecimate) != 0))
                    {
                        particles[j].energy = 0; // energy set to 0 remove the particle, as per Unity doc
                    }
                }

                if (particles[j].energy > 0)
                {
                    Vector3d pPos = persistentEmitters[i].pe.useWorldSpace
                                        ? particles[j].position
                                        : persistentEmitters[i].pe.transform.TransformPoint(particles[j].position);
                    Vector3d pVel = (persistentEmitters[i].pe.useWorldSpace
                                         ? particles[j].velocity
                                         : persistentEmitters[i].pe.transform.TransformDirection(particles[j].velocity))
                                    + Krakensbane.GetFrameVelocity();

                    // try-finally block to ensure we set the particle velocities correctly in the end.
                    try
                    {
                        // Fixed update is not the best place to update the size but the particles array copy is slow so doing each frame would be worse

                        // No need to waste time doing a division if the result is 0.
                        if (logarithmicGrow != 0.0)
                        {
                            // Euler integration of the derivative of Log(logarithmicGrowth * t + 1) + 1.
                            // This might look weird.
                            particles[j].size +=
                                (float)
                                (((TimeWarp.fixedDeltaTime * logarithmicGrow)
                                  / (1 + (particles[j].startEnergy - particles[j].energy) * logarithmicGrow))
                                 * averageSize);
                        }

                        particles[j].size = Mathf.Min(particles[j].size, sizeClamp);

                        if (particles[j].energy == particles[j].startEnergy)
                        {
                            if (fixedEmissions)
                            {
                                // Uniformly scatter newly emitted particles along the emitter's trajectory in order to remove the dotted smoke effect.
                                // use variableDeltaTime since the particle are emited on Update anyway.
                                pPos -= (hostPart.rb.velocity + Krakensbane.GetFrameVelocity())
                                        * UnityEngine.Random.value * variableDeltaTime;
                            }
                            if (randomInitalVelocityOffsetMaxRadius != 0.0)
                            {
                                Vector2 diskPoint = UnityEngine.Random.insideUnitCircle
                                                    * randomInitalVelocityOffsetMaxRadius;
                                Vector3d offset;
                                if (pVel.x == 0.0 && pVel.y == 0.0)
                                {
                                    offset = new Vector3d(diskPoint.x, diskPoint.y, 0.0);
                                }
                                else
                                {
                                    // Convoluted calculations to save some operations (especially divisions).
                                    // Not that it really matters, but this achieves 2 divisions and 1 square root.
                                    double x = pVel.x;
                                    double y = pVel.y;
                                    double z = pVel.z;
                                    double xSquared = x * x;
                                    double ySquared = y * y;
                                    double xySquareNorm = xSquared + ySquared;
                                    double inverseXYSquareNorm = 1 / xySquareNorm;
                                    double inverseNorm = 1 / Math.Sqrt(xySquareNorm + z * z);
                                    double zOverNorm = z * inverseNorm;
                                    // TODO(robin): find an identifier for that...
                                    double mixedTerm = x * y * (zOverNorm - 1);
                                    offset =
                                        new Vector3d(
                                            ((ySquared + xSquared * zOverNorm) * diskPoint.x + mixedTerm * diskPoint.y)
                                            * inverseXYSquareNorm,
                                            ((xSquared + ySquared * zOverNorm) * diskPoint.y + mixedTerm * diskPoint.x)
                                            * inverseXYSquareNorm,
                                            -(x * diskPoint.x + y * diskPoint.y) * inverseNorm);
                                }
                                pVel += offset;
                            }
                        }

                        if (physical && !SmokeScreenConfig.Instance.globalPhysicalDisable
                            && (j % physicsPass == activePhysicsPass))
                        {
                            // There must be a way to keep the actual initial volume, 
                            // but I'm lazy.
                            pVel = ParticlePhysics(particles[j].size, averageSize, pPos, pVel);
                        }

                        if (collide && !SmokeScreenConfig.Instance.globalCollideDisable
                            && particles[j].energy != particles[j].startEnergy
                            // Do not collide newly created particles (they collide with the emitter and things look bad).
                            && (j % physicsPass == activePhysicsPass))
                        {
                            pVel = ParticleCollision(pPos, pVel, hit, mask);
                        }
                    }
                    finally
                    {
                        particles[j].velocity = (persistentEmitters[i].pe.useWorldSpace ?
                                              (Vector3)(pVel - Krakensbane.GetFrameVelocity()) :
                                              persistentEmitters[i].pe.transform.InverseTransformDirection(pVel - Krakensbane.GetFrameVelocity()));
                        particles[j].position = persistentEmitters[i].pe.useWorldSpace
                                                    ? (Vector3)pPos
                                                    : persistentEmitters[i].pe.transform.InverseTransformPoint(pPos);
                    }
                }
            }
            activePhysicsPass = ++activePhysicsPass % physicsPass;
            persistentEmitters[i].pe.pe.particles = particles;
            SmokeScreenConfig.activeParticles += persistentEmitters[i].pe.pe.particleCount;
        }
    }

    private Vector3 ParticlePhysics(double radius, double initialRadius, Vector3d pPos, Vector3d pVel)
    {
        // N.B.: multiplications rather than Pow, Pow is slow,
        // multiplication by .5 rather than division by 2 (same 
        // reason).
        CelestialBody mainBody = FlightGlobals.currentMainBody;
        double estimatedInitialVolume = 0.75 * Math.PI * initialRadius * initialRadius * initialRadius;
        double currentVolume = 0.75 * Math.PI * radius * radius * radius;
        double volumeChange = currentVolume - estimatedInitialVolume;
        double atmosphericDensity = FlightGlobals.getAtmDensity(FlightGlobals.getStaticPressure(pPos, mainBody));
        double density = (estimatedInitialVolume * initialDensity + volumeChange * atmosphericDensity) / currentVolume;
        double mass = density * currentVolume;

        // Weight and buoyancy.
        Vector3d mainBodyDist = mainBody.position - pPos;
        Vector3d geeForce = mainBodyDist.normalized * (mainBody.gMagnitudeAtCenter / mainBodyDist.sqrMagnitude);
        Vector3d acceleration = (1 - (atmosphericDensity / density)) * geeForce;

        // Drag. TODO(robin): simplify.
        acceleration += -0.5 * atmosphericDensity * pVel * pVel.magnitude * dragCoefficient * Math.PI * radius * radius
                        / mass;

        // Euler is good enough for graphics.
        return pVel + acceleration * TimeWarp.fixedDeltaTime * (float)physicsPass;
    }

    private Vector3 ParticleCollision(Vector3d pPos, Vector3d pVel, RaycastHit hit, int mask)
    {
        if (Physics.Raycast(
            pPos,
            pVel,
            out hit,
            (float)pVel.magnitude * TimeWarp.fixedDeltaTime * (float)physicsPass,
            mask))
        {
            //// collidersName[hit.collider.name] = true;

            if (hit.collider.name != LaunchPadGrateColliderName)
            {
                Vector3 unitTangent = (hit.normal.x == 0 && hit.normal.y == 0)
                                          ? new Vector3(1, 0, 0)
                                          : Vector3.Exclude(hit.normal, new Vector3(0, 0, 1)).normalized;
                Vector3 hVel = Vector3.Exclude(hit.normal, pVel);
                Vector3 reflectedNormalVelocity = hVel - pVel;
                float residualFlow = reflectedNormalVelocity.magnitude * (1 - collideRatio);

                // An attempt at a better velocity change; the blob collides with some
                // restitution coefficient collideRatio << 1 and we add a random tangential term
                // for outflowing particles---randomness handwaved in through fluid dynamics:
                float randomAngle = UnityEngine.Random.value * 360.0f;
                Vector3d outflow = Quaternion.AngleAxis(randomAngle, hit.normal) * unitTangent * residualFlow;
                pVel = hVel + collideRatio * reflectedNormalVelocity + outflow * (1 - stickiness);
            }
            else
            {
                // Don't collide with the launch pad grid and add colliders under it
                if (!addedLaunchPadCollider)
                {
                    AddLaunchPadColliders(hit);
                }
            }
        }
        return pVel;
    }

    // The whole pad object is named "ksp_pad_launchPad"
    private const string LaunchPadGrateColliderName = "Launch Pad Grate";

    private const string LaunchPadColliderName = "LaunchPadColliderSmokeScreen";

    private void AddLaunchPadColliders(RaycastHit hit)
    {
        // the Grate Collider size is  (37.70, 20.22, 3.47). Way larger that the actual grate
        // The current collider do not cover all this area. More are needed

        Transform parentTransform = hit.collider.gameObject.transform;

        ////print("AddLaunchPadColliders col name = " + hit.collider.gameObject.name);

        ////print("AddLaunchPadColliders parent col name = " + hit.collider.gameObject.transform.parent.gameObject.name);

        // Are the collider already here ?
        if (parentTransform.FindChild(LaunchPadColliderName))
        {
            addedLaunchPadCollider = true;
            return;
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

        addedLaunchPadCollider = true;
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
        if (hostPart != null)
        {
            partTemp = hostPart.temperature;

            if (hostPart.vessel != null)
            {
                Vessel vessel = hostPart.vessel;
                atmDensity = (float)vessel.atmDensity;

                externalTemp = vessel.flightIntegrator.getExternalTemperature();

                // FAR use a nice config file to get the atmo info for each body. 
                // For now I'll just use Air for all.
                const double magicNumberFromFAR = 1.4 * 8.3145 * 1000 / 28.96;
                double speedOfSound = Math.Sqrt((externalTemp + 273.15) * magicNumberFromFAR);
                surfaceVelMach = (float)(vessel.srf_velocity.magnitude / speedOfSound);
            }
            else
            {
                atmDensity =
                    (float)
                    FlightGlobals.getAtmDensity(
                        FlightGlobals.getStaticPressure(hostPart.transform.position, FlightGlobals.currentMainBody));
                externalTemp = FlightGlobals.getExternalTemperature(hostPart.transform.position);
                const double magicNumberFromFAR = 1.4 * 8.3145 * 1000 / 28.96;
                double speedOfSound = Math.Sqrt((externalTemp + 273.15) * magicNumberFromFAR);
                surfaceVelMach =
                    (float)
                    ((hostPart.vel - FlightGlobals.currentMainBody.getRFrmVel(hostPart.transform.position)).magnitude
                     / speedOfSound);
            }
        }

        inputs[(int)MultiInputCurve.Inputs.power] = power;
        inputs[(int)MultiInputCurve.Inputs.density] = atmDensity;
        inputs[(int)MultiInputCurve.Inputs.mach] = surfaceVelMach;
        inputs[(int)MultiInputCurve.Inputs.parttemp] = partTemp;
        inputs[(int)MultiInputCurve.Inputs.externaltemp] = externalTemp;
    }

    public void UpdateEmitters(float power)
    {
        UpdateInputs(power);

        for (int i = 0; i < persistentEmitters.Count; i++)
        {
            PersistentKSPParticleEmitter pkpe = persistentEmitters[i];

            float sizePower = size.Value(inputs) * fixedScale;
            pkpe.pe.minSize = Mathf.Min(pkpe.minSizeBase * sizePower, sizeClamp);
            pkpe.pe.maxSize = Mathf.Min(pkpe.maxSizeBase * sizePower, sizeClamp);

            float emissionPower = emission.Value(inputs);
            pkpe.pe.minEmission = Mathf.FloorToInt(pkpe.minEmissionBase * emissionPower);
            pkpe.pe.maxEmission = Mathf.FloorToInt(pkpe.maxEmissionBase * emissionPower);

            float energyPower = energy.Value(inputs);
            pkpe.pe.minEnergy = pkpe.minEnergyBase * energyPower;
            pkpe.pe.maxEnergy = pkpe.maxEnergyBase * energyPower;

            float velocityPower = speed.Value(inputs);
            pkpe.pe.localVelocity = pkpe.localVelocityBase * velocityPower;
            pkpe.pe.worldVelocity = pkpe.worldVelocityBase * velocityPower;

            float forcePower = force.Value(inputs);
            pkpe.pe.force = pkpe.forceBase * forcePower;

            pkpe.pe.sizeGrow = grow.Value(inputs);

            float currentScale = scale.Value(inputs) * fixedScale;
            pkpe.pe.shape1D = pkpe.scale1DBase * currentScale;
            pkpe.pe.shape2D = pkpe.scale2DBase * currentScale;
            pkpe.pe.shape3D = pkpe.scale3DBase * currentScale;

            logarithmicGrow = logGrow.Value(inputs);

            pkpe.go.transform.localPosition = localPosition
                                              + offsetDirection.normalized * offset.Value(inputs) * fixedScale;

            pkpe.go.transform.localRotation = Quaternion.Euler(localRotation);

            ////print(atmDensity.ToString("F2") + " " + offset.Value(power).ToString("F2") + " " + offsetFromDensity.Value(atmDensity).ToString("F2") + " " + offsetFromMach.Value(surfaceVelMach).ToString("F2"));
        }
    }

    public void Update()
    {
        variableDeltaTime = Time.deltaTime; // used for the fixedemission particles distribution
        if (persistentEmitters == null)
        {
            return;
        }
        for (int i = 0; i < persistentEmitters.Count; i++)
        {
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
        if (shaderFileName != string.Empty && System.IO.File.Exists(filename))
        {
            try
            {
                System.IO.TextReader shaderFile = new System.IO.StreamReader(filename);
                string shaderText = shaderFile.ReadToEnd();
                shader = new Material(shaderText).shader;
            }
            catch (Exception e)
            {
                print("unable to load shader " + shaderFileName + " : " + e.ToString());
            }
        }

        List<Transform> transforms = new List<Transform>(hostPart.FindModelTransforms(transformName));
        if (transforms.Count == 0)
        {
            print("Cannot find transform " + transformName);
            return;
        }
        GameObject model = GameDatabase.Instance.GetModel(modelName);
        if (model == null)
        {
            print("Cannot find model " + modelName);
            return;
        }
        model.SetActive(true);
        KSPParticleEmitter templateKspParticleEmitter = model.GetComponentInChildren<KSPParticleEmitter>();

        if (templateKspParticleEmitter == null)
        {
            print("Cannot find particle emitter on " + modelName);
            UnityEngine.Object.Destroy(model);
            return;
        }

        if (shader != null)
        {
            templateKspParticleEmitter.material.shader = shader;
        }

        fixedEmissions = templateKspParticleEmitter.useWorldSpace;

        if (persistentEmitters == null)
        {
            persistentEmitters = new List<PersistentKSPParticleEmitter>();
        }

        for (int i = 0; i < transforms.Count; i++)
        {
            GameObject emitterGameObject = UnityEngine.Object.Instantiate(model) as GameObject;
            KSPParticleEmitter childKSPParticleEmitter = emitterGameObject.GetComponentInChildren<KSPParticleEmitter>();

            if (childKSPParticleEmitter != null)
            {
                PersistentKSPParticleEmitter pkpe = new PersistentKSPParticleEmitter(
                    emitterGameObject,
                    childKSPParticleEmitter,
                    templateKspParticleEmitter);

                try
                {
                    childKSPParticleEmitter.particleRenderMode =
                        (ParticleRenderMode)Enum.Parse(typeof(ParticleRenderMode), renderMode);
                }
                catch (ArgumentException)
                {
                    print("ModelMultiParticleFXExt: " + renderMode + " is not a valid ParticleRenderMode");
                }

                persistentEmitters.Add(pkpe);

                emitterGameObject.transform.SetParent(transforms[i]);

                emitterGameObject.transform.localPosition = localPosition;
                emitterGameObject.transform.localRotation = Quaternion.Euler(localRotation);
            }
        }

        UnityEngine.Object.Destroy(templateKspParticleEmitter);

        list.Add(this);
    }

    public void Backup(ConfigNode node)
    {
        node_backup = SmokeScreenUtil.WriteRootNode(node);
        //print("Backup node_backup is\n " + node_backup.Replace(Environment.NewLine, Environment.NewLine + "ModelMultiParticlePersistFX "));
    }

    public void Restore()
    {
        //print("Restore node_backup is\n " + node_backup.Replace(Environment.NewLine, Environment.NewLine + "ModelMultiParticlePersistFX "));
        string[] text = node_backup.Split(new string[] { "\n" }, StringSplitOptions.None);
        ConfigNode node = SmokeScreenUtil.RecurseFormat(SmokeScreenUtil.PreFormatConfig(text));
        this.OnLoad(node);
    }

    public override void OnLoad(ConfigNode node)
    {
        //print("OnLoad");
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

        ConfigNode.LoadObjectFromConfig(this, node);
        emission.Load(node);
        energy.Load(node);
        speed.Load(node);
        grow.Load(node);
        scale.Load(node);
        size.Load(node);
        offset.Load(node);
        force.Load(node);

        logGrow.Load(node);

        angle.Load("angle", node);
        distance.Load("distance", node);
    }

    public override void OnSave(ConfigNode node)
    {
        ConfigNode.CreateConfigFromObject(this, node);
        emission.Save(node);
        energy.Save(node);
        speed.Save(node);
        grow.Save(node);
        scale.Save(node);
        size.Save(node);
        offset.Save(node);
        force.Save(node);
        logGrow.Save(node);

        angle.Save(node);
        distance.Save(node);
    }

    private static void print(String s)
    {
        MonoBehaviour.print("[ModelMultiParticlePersistFX] " + s);
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
        if (showUI)
        {
            winPos = GUILayout.Window(
                winID,
                winPos,
                windowGUI,
                hostPart.name + " " + this.effectName + " " + this.instanceName,
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

        GUILayout.Space(10);

        if (GUILayout.Button("Clear Particles"))
        {
            for (int i = 0; i < persistentEmitters.Count; i++)
            {
                persistentEmitters[i].pe.pe.ClearParticles();
            }
        }

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
                this.OnSave(node);
                nodeText = SmokeScreenUtil.WriteRootNode(node);
            }

            // Apply the text
            if (GUILayout.Button("Apply"))
            {
                string[] text = nodeText.Split(new string[] { "\n" }, StringSplitOptions.None);
                ConfigNode node = SmokeScreenUtil.RecurseFormat(SmokeScreenUtil.PreFormatConfig(text));
                this.OnLoad(node);
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
            text + " Val=" + inputs[id].ToString("F3") + " Min=" + min.ToString("F2") + " Max=" + max.ToString("F2"));

        if (overRideInputs)
        {
            GUILayout.BeginHorizontal();
            boxInput[id] = GUILayout.Toggle(boxInput[id], "", GUILayout.ExpandWidth(false));

            if (boxInput[id])
            {
                float.TryParse(
                    GUILayout.TextField(inputs[id].ToString("F2"), GUILayout.ExpandWidth(true), GUILayout.Width(100)),
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

        return max;
    }
}