/*
 * Author: Sébastien GAGGINI AKA Sarbian, France
 * License: Attribution 4.0 International (CC BY 4.0): http://creativecommons.org/licenses/by/4.0/
 * 
 * Thanks to Nothke for all the feature ideas, testing and feedback
 * 
 */
using System;
using System.Collections.Generic;
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

    // Logarithmic growth applied to to the particle.
    // The size at time t after emission will be approximately
    // (Log(logarithmicGrowth * t + 1) + 1) * initialSize, assuming growth = 0.
    // TODO(sarbian): make this a cfg-configurable curve (as a function of 
    // density).
    [Persistent]
    public double logarithmicGrowth = 0.0;


    /// <summary>
    /// Whether to nudge particles in order to alleviate the dotted smoke effect.
    /// Set this to true (default) when using 'Simulate World Space' in Unity,
    /// false otherwise.
    /// </summary>
    [Persistent]
    public bool fixedEmissions = true;

    // The initial velocity of the particles will be offset by a random amount
    // lying in a disk perpendicular to the mean initial velocity whose radius
    // is randomOffsetMaxRadius. This is similar to Unity's 'Random Velocity'
    // Setting, except it will sample the offset from a (normal) disk rather
    // than from a cube. Units (SI): m/s.
    [Persistent]
    public float randomInitalVelocityOffsetMaxRadius = 0.0f;
    #endregion Persistent fields

    public MultiInputCurve emission = new MultiInputCurve("emission");
    public MultiInputCurve energy   = new MultiInputCurve("energy");
    public MultiInputCurve speed    = new MultiInputCurve("speed");
    public MultiInputCurve grow     = new MultiInputCurve("grow", true);
    public MultiInputCurve scale    = new MultiInputCurve("scale");
    public MultiInputCurve size     = new MultiInputCurve("size");
    public MultiInputCurve offset   = new MultiInputCurve("offset", true);

    public MultiInputCurve logGrow  = new MultiInputCurve("loggrow", true);

    // Those 2 curve are related to the angle and distance to cam
    public FXCurve angle = new FXCurve("angle", 1f);
    public FXCurve distance = new FXCurve("distance", 1f);

    private List<PersistentKSPParticleEmitter> persistentEmitters;

    private float lastPower;

    private float minEmissionBase;
    private float maxEmissionBase;

    private float minEnergyBase;
    private float maxEnergyBase;

    private float minSizeBase;
    private float maxSizeBase;

    private float scale1DBase;
    private Vector2 scale2DBase;
    private Vector3 scale3DBase;

    private Vector3 localVelocityBase;

    private Shader shader;

    public static int activeParticles = 0;
    public static int particuleDecimate = 0;
    public static uint particleCounter = 0;

    // Particule Emitter with more than decimateFloor particules will have
    // some particle culled if there is more than maximumActiveParticles active
    public static int decimateFloor = 30;
    public static int maximumActiveParticles = 8000; // The engine won't spawn more than 10k anyway

    private void OnDestroy()
    {
        if (persistentEmitters != null)
        {
            for (int i = 0; i < persistentEmitters.Count; i++)
                persistentEmitters[i].Detach(0);
        }
    }

    public override void OnEvent()
    {
        if (persistentEmitters == null)
        {
            return;
        }

        UpdateEmitters(1);
        for (int i = 0; i < persistentEmitters.Count; i++)
            persistentEmitters[i].pe.Emit();
    }

    public override void OnEvent(float power)
    {
        if (persistentEmitters == null)
            return;

        lastPower = power;

        if (power > 0f)
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

    private List<Component> partColliders = new List<Component>();

    bool addedLaunchPadCollider = false;

    private Dictionary<string, bool> colidersName;

    public static uint physicsPass = 4;
    public static uint activePhysicsPass = 0;

    public void FixedUpdate()
    {
        if (persistentEmitters == null)
        {
            return;
        }

        ResetParticleCount();

        if (colidersName == null)
            colidersName = new Dictionary<string, bool>(StringComparer.Ordinal);


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
                double averageEmittedParticles = UnityEngine.Random.Range(persistentEmitters[i].pe.minEmission, persistentEmitters[i].pe.maxEmission) * TimeWarp.fixedDeltaTime;
                int emittedParticles = (int)Math.Floor(averageEmittedParticles) + (UnityEngine.Random.value < averageEmittedParticles - Math.Floor(averageEmittedParticles) ? 1 : 0);
                for (int k = 0; k < emittedParticles; ++k)
                {
                    persistentEmitters[i].pe.EmitParticle();
                }
            }

            // This line (and the one that does the oposite at the end) is actally the slowest part of the whole function 
            Particle[] particles = persistentEmitters[i].pe.pe.particles;

            for (int j = 0; j < particles.Length; j++)
            {
                // Check if we need to cull the number of particles
                if (particuleDecimate != 0 && particles.Length > decimateFloor)
                {
                    particleCounter++;
                    if ((particuleDecimate > 0 && (particleCounter % particuleDecimate) == 0)
                        || (particuleDecimate < 0 && (particleCounter % particuleDecimate) != 0))
                    {
                        particles[j].energy = 0; // energy set to 0 remove the particle, as per Unity doc
                    }
                }

                if (particles[j].energy > 0)
                {
                    Vector3d pPos = persistentEmitters[i].pe.useWorldSpace ? particles[j].position : persistentEmitters[i].pe.transform.TransformPoint(particles[j].position);
                    Vector3d pVel = (persistentEmitters[i].pe.useWorldSpace ? particles[j].velocity : persistentEmitters[i].pe.transform.TransformDirection(particles[j].velocity))
                                    + Krakensbane.GetFrameVelocity();

                    // try-finally block to ensure we set the particle velocities correctly in the end.
                    try
                    {
                        // Fixed update is not the best place to update the size but the particles array copy is slow so doing each frame would be worse
                        particles[j].size = Mathf.Min(particles[j].size, sizeClamp);

                        var inputs = this.GetInputs(lastPower);

                        float lGrowth = logGrow.Value(inputs);
                        
                        // No need to waste time doing a division if the result is 0.
                        if (lGrowth != 0.0)
                        {
                            // Euler integration of the derivative of Log(logarithmicGrowth * t + 1) + 1.
                            // This might look weird.
                            particles[j].size += (float)(((TimeWarp.fixedDeltaTime * lGrowth) / (1 + (particles[j].startEnergy - particles[j].energy) * lGrowth)) * 0.5 * (persistentEmitters[i].pe.minSize + persistentEmitters[i].pe.maxSize));
                        }

                        if (particles[j].energy == particles[j].startEnergy)
                        {
                            if (fixedEmissions)
                            {
                                // Uniformly scatter newly emitted particles along the emitter's trajectory in order to remove the dotted smoke effect.
                                pPos -= (hostPart.rb.velocity + Krakensbane.GetFrameVelocity()) * UnityEngine.Random.value * TimeWarp.fixedDeltaTime;
                            }
                            if (randomInitalVelocityOffsetMaxRadius != 0.0)
                            {
                                Vector2 diskPoint = UnityEngine.Random.insideUnitCircle * randomInitalVelocityOffsetMaxRadius;
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
                                    offset = new Vector3d(
                                        ((ySquared + xSquared * zOverNorm) * diskPoint.x + mixedTerm * diskPoint.y) * inverseXYSquareNorm,
                                        ((xSquared + ySquared * zOverNorm) * diskPoint.y + mixedTerm * diskPoint.x) * inverseXYSquareNorm,
                                        -(x * diskPoint.x + y * diskPoint.y) * inverseNorm);
                                }
                                pVel += offset;
                            }
                        }

                        if (physical && (j % physicsPass == activePhysicsPass))
                        {
                            // TODO(robin): using pe.minSize is probably a bad idea, as above.
                            // There must be a way to keep the actual initial volume, 
                            // but I'm lazy.
                            pVel = ParticlePhysics(particles[j].size, 0.5  * (persistentEmitters[i].pe.minSize + persistentEmitters[i].pe.maxSize), pPos, pVel);
                        }

                        if (collide && particles[j].energy != particles[j].startEnergy  // Do not collide newly created particles (they collide with the emitter and things look bad).
                            && (j % physicsPass == activePhysicsPass))
                            //&& pad != null && (pad.transform.position - pPos).sqrMagnitude < 25f * 25f) // Dont colide if the particule is far from the pad
                        {
                            pVel = ParticleCollision(pPos, pVel, hit, mask);
                        }
                    }
                    finally
                    {
                        particles[j].velocity = (persistentEmitters[i].pe.useWorldSpace ? (Vector3)pVel : persistentEmitters[i].pe.transform.InverseTransformDirection(pVel)) 
                                                    - Krakensbane.GetFrameVelocity();
                        particles[j].position = persistentEmitters[i].pe.useWorldSpace ? (Vector3)pPos : persistentEmitters[i].pe.transform.InverseTransformPoint(pPos);
                    }
                }
            }
            activePhysicsPass = ++activePhysicsPass % physicsPass;
            persistentEmitters[i].pe.pe.particles = particles;
            activeParticles += persistentEmitters[i].pe.pe.particleCount;
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
        acceleration += -0.5 * atmosphericDensity * pVel * pVel.magnitude * dragCoefficient * Math.PI * radius * radius / mass;

        // Euler is good enough for graphics.
        return pVel + acceleration * TimeWarp.fixedDeltaTime * (float)physicsPass;
    }

    private Vector3 ParticleCollision(Vector3d pPos, Vector3d pVel, RaycastHit hit, int mask)
    {
        if (Physics.Raycast(pPos, pVel, out hit, (float)pVel.magnitude * TimeWarp.fixedDeltaTime * (float)physicsPass, mask))
        {
            //// colidersName[hit.collider.name] = true;

            if (hit.collider.name != LaunchPadGrateColliderName)
            {

                Vector3 unitTangent = (hit.normal.x == 0 && hit.normal.y == 0) ? new Vector3(1, 0, 0) : Vector3.Exclude(hit.normal, new Vector3(0, 0, 1)).normalized;
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
                    AddLaunchPadColliders(hit);
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


    private static float lastTime = 0;
    private static void ResetParticleCount()
    {
        if (lastTime < Time.fixedTime)
        {
            if (activeParticles > maximumActiveParticles)
            {
                int toRemove = activeParticles - maximumActiveParticles;
                if (toRemove < maximumActiveParticles)
                    particuleDecimate = activeParticles / (toRemove + 1); // positive we remove each n
                else
                    particuleDecimate = -activeParticles / maximumActiveParticles; // negative we keep each n
            }
            else
                particuleDecimate = 0;

            //print(activeParticles + " " + particuleDecimate + " " + particleCounter);

            activeParticles = 0;
            lastTime = Time.fixedTime;
        }
    }

    private Dictionary<MultiInputCurve.Inputs, float> GetInputs(float power)
    {
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

                // does not work
                //float p = (float)FlightGlobals.getStaticPressure(vessel.altitude, vessel.mainBody);
                //float speedOfSound = Mathf.Sqrt(1.4f * p / atmDensity);

                // Cheating for now
                const double speedOfSound = 343.2f;

                //double speedOfSound = Math.Sqrt(externalTemp * currentBodyAtm.x)

                surfaceVelMach = (float)(vessel.srf_velocity.magnitude / speedOfSound);
            }
            else
            {
                // TODO atmDensity & mach when not attached to a vessel
                atmDensity = (float)FlightGlobals.getAtmDensity(FlightGlobals.getStaticPressure(hostPart.transform.position, FlightGlobals.currentMainBody));
            }
        }

        var inputs = new Dictionary<MultiInputCurve.Inputs, float>(5);
        inputs[MultiInputCurve.Inputs.power] = power;
        inputs[MultiInputCurve.Inputs.density] = atmDensity;
        inputs[MultiInputCurve.Inputs.mach] = surfaceVelMach;
        inputs[MultiInputCurve.Inputs.parttemp] = partTemp;
        inputs[MultiInputCurve.Inputs.externaltemp] = externalTemp;

        return inputs;
    }


    public void UpdateEmitters(float power)
    {
        var inputs = this.GetInputs(power);

        for (int i = 0; i < persistentEmitters.Count; i++)
        {
            float sizePower = size.Value(inputs);
            persistentEmitters[i].pe.minSize = Mathf.Min(minSizeBase * sizePower, sizeClamp);
            persistentEmitters[i].pe.maxSize = Mathf.Min(maxSizeBase * sizePower, sizeClamp);

            float emissionPower = emission.Value(inputs);
            persistentEmitters[i].pe.minEmission = Mathf.FloorToInt(minEmissionBase * emissionPower);
            persistentEmitters[i].pe.maxEmission = Mathf.FloorToInt(maxEmissionBase * emissionPower);

            float energyPower = energy.Value(inputs);
            persistentEmitters[i].pe.minEnergy = minEnergyBase * energyPower;
            persistentEmitters[i].pe.maxEnergy = maxEnergyBase * energyPower;

            float localVelocityPower = speed.Value(inputs);
            persistentEmitters[i].pe.localVelocity = localVelocityBase * localVelocityPower;

            persistentEmitters[i].pe.sizeGrow = grow.Value(inputs);

            float currentScale = scale.Value(inputs);
            persistentEmitters[i].pe.shape1D = scale1DBase * currentScale;
            persistentEmitters[i].pe.shape2D = scale2DBase * currentScale;
            persistentEmitters[i].pe.shape3D = scale3DBase * currentScale;


            persistentEmitters[i].go.transform.localPosition = Vector3d.forward * offset.Value(inputs);

            ////print(atmDensity.ToString("F2") + " " + offset.Value(power).ToString("F2") + " " + offsetFromDensity.Value(atmDensity).ToString("F2") + " " + offsetFromMach.Value(surfaceVelMach).ToString("F2"));
        }
    }

    public void Update()
    {
        if (persistentEmitters == null)
        {
            return;
        }
        for (int i = 0; i < persistentEmitters.Count; i++)
        {
            // using Camera.main will mess up anything multi cam but using current require adding a OnWillRenderObject() to the ksp particle emitter GameObject (? not tested)
            float currentAngle = Vector3.Angle(-Camera.main.transform.forward, persistentEmitters[i].go.transform.forward);
            float currentDist = (Camera.main.transform.position - persistentEmitters[i].go.transform.position).magnitude;

            persistentEmitters[i].pe.maxParticleSize = persistentEmitters[i].baseMaxSize * angle.Value(currentAngle) * distance.Value(currentDist);
            persistentEmitters[i].pe.pr.maxParticleSize = persistentEmitters[i].pe.maxParticleSize;
        }
    }


    public override void OnInitialize()
    {
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


        // TODO : move those in PersistentKSPParticleEmitter 
        scale1DBase = (templateKspParticleEmitter.shape1D *= fixedScale);
        scale2DBase = (templateKspParticleEmitter.shape2D *= fixedScale);
        scale3DBase = (templateKspParticleEmitter.shape3D *= fixedScale);

        minEmissionBase = (float)templateKspParticleEmitter.minEmission;
        maxEmissionBase = (float)templateKspParticleEmitter.maxEmission;
        minEnergyBase = templateKspParticleEmitter.minEnergy;
        maxEnergyBase = templateKspParticleEmitter.maxEnergy;

        minSizeBase = (float)templateKspParticleEmitter.minSize;
        maxSizeBase = (float)templateKspParticleEmitter.maxSize;

        localVelocityBase = templateKspParticleEmitter.localVelocity;

        if (persistentEmitters == null)
            persistentEmitters = new List<PersistentKSPParticleEmitter>();

        for (int i = 0; i < transforms.Count; i++)
        {
            GameObject emmitterGameObject = UnityEngine.Object.Instantiate(model) as GameObject;
            KSPParticleEmitter childKSPParticleEmitter = emmitterGameObject.GetComponentInChildren<KSPParticleEmitter>();

            if (childKSPParticleEmitter != null)
            {

                PersistentKSPParticleEmitter pkpe = new PersistentKSPParticleEmitter(emmitterGameObject, childKSPParticleEmitter, templateKspParticleEmitter.maxParticleSize);

                childKSPParticleEmitter.shape1D *= fixedScale;
                childKSPParticleEmitter.shape2D *= fixedScale;
                childKSPParticleEmitter.shape3D *= fixedScale;

                try
                {
                    childKSPParticleEmitter.particleRenderMode = (ParticleRenderMode)Enum.Parse(typeof(ParticleRenderMode), renderMode);
                }
                catch (ArgumentException)
                {
                    print("ModelMultiParticleFXExt: " + renderMode + " is not a valid ParticleRenderMode");
                }

                persistentEmitters.Add(pkpe);


                emmitterGameObject.transform.SetParent(transforms[i]);

                emmitterGameObject.transform.localPosition = localPosition;
                emmitterGameObject.transform.localRotation = Quaternion.Euler(localRotation);
            }

        }

        UnityEngine.Object.Destroy(templateKspParticleEmitter);
    }

    public override void OnLoad(ConfigNode node)
    {
        ConfigNode.LoadObjectFromConfig(this, node);
        emission.Load(node);
        energy.Load(node);
        speed.Load(node);
        grow.Load(node);
        scale.Load(node);
        size.Load(node);
        offset.Load(node);

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

        angle.Save(node);
        distance.Save(node);
    }

    private void print(String s)
    {
        MonoBehaviour.print(this.GetType().Name + " : " + s);
    }

    Rect winPos = new Rect(300, 100, 300, 100);

    int wid = 0;

    void OnGUI()
    {
        if (!HighLogic.LoadedSceneIsFlight)
            return;

        if (wid == 0)
            wid = UnityEngine.Random.Range(30000, 40000);

        winPos = GUILayout.Window(wid, winPos, windowGUI, "ModelMultiParticlePersistFX", GUILayout.MinWidth(300));
    }

    void windowGUI(int ID)
    {
        GUILayout.BeginVertical();

        collide = GUILayout.Toggle(collide, "collide");
        physical = GUILayout.Toggle(physical, "physical");

        GUILayout.Label("activePhysicPass " + activePhysicsPass);

        GUILayout.Space(10);

        GUILayout.BeginHorizontal();
        GUILayout.Label("maximumActiveParticles", GUILayout.ExpandWidth(true));
        maximumActiveParticles = int.Parse(GUILayout.TextField(maximumActiveParticles.ToString(), GUILayout.ExpandWidth(true), GUILayout.Width(100)));
        GUILayout.EndHorizontal();

        GUILayout.Label("activeParticles : " + activeParticles);
        GUILayout.Label("particuleDecimate : " + particuleDecimate);
        GUILayout.Label("particleCounter : " + particleCounter);
                
        GUILayout.Space(10);

        double externalTemp = FlightGlobals.ActiveVessel.flightIntegrator.getExternalTemperature();
        GUILayout.Label("External Temperature : " + externalTemp.ToString("F2"));


        // FAR use a nice config file to get the atmo info for each body. 
        // For now I'll just use Air for all.
        const double magicNumeberFromFAR = 1.4 * 8.3145 * 1000 / 28.96;
        double speedOfSound = Math.Sqrt((externalTemp + 273.15) * magicNumeberFromFAR);

        GUILayout.Label("Speed of Sound : " + speedOfSound.ToString("F2"));

        GameObject pad = GameObject.Find("ksp_pad_launchPad");

        if (pad != null)
        {
            GUILayout.Label("Scale : " + pad.transform.localScale.magnitude.ToString("F2"));
            GUILayout.Space(10);
        }

        if (GUILayout.Button("Reset"))
            colidersName = new Dictionary<string, bool>(StringComparer.Ordinal);

        if (colidersName != null)
            foreach (string s in colidersName.Keys)
                GUILayout.Label(s);

        GUILayout.EndVertical();

        GUI.DragWindow();
    }

}
