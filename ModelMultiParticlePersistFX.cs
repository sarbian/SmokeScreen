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
    [Persistent]
    public string modelName = string.Empty;

    [Persistent]
    public string transformName = string.Empty;

    [Persistent]
    public string shaderFileName = string.Empty;

    [Persistent]
    public string renderMode = "Billboard";

    [Persistent]
    public string collide = "false";

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

    // Whether to apply Archimedes' force, gravity and other things to the particle.
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
    // TODO(sarbian): make this a cfg-configurable curve (as a function of density).
    [Persistent]
    public double logarithmicGrowth = 0.0;
    
    // Whether to nudge particles in order to alleviate the dotted smoke effect.
    [Persistent]
    public bool fixedEmissions = true;

    private float variableDeltaTime;
    
    public FXCurve emission = new FXCurve("emission", 1f);

    public FXCurve energy = new FXCurve("energy", 1f);

    public FXCurve speed = new FXCurve("speed", 1f);

    public FXCurve grow = new FXCurve("grow", 0f);

    public FXCurve scale = new FXCurve("scale", 1f);

    public FXCurve size = new FXCurve("size", 1f);

    public FXCurve offset = new FXCurve("offset", 0f);


    public FXCurve emissionFromDensity = new FXCurve("density", 1f);

    public FXCurve energyFromDensity = new FXCurve("density", 1f);

    public FXCurve speedFromDensity = new FXCurve("density", 1f);

    public FXCurve growFromDensity = new FXCurve("density", 0f);

    public FXCurve scaleFromDensity = new FXCurve("density", 1f);

    public FXCurve sizeFromDensity = new FXCurve("density", 1f);

    public FXCurve offsetFromDensity = new FXCurve("density", 0f);

    public FXCurve emissionFromMach = new FXCurve("mach", 1f);

    public FXCurve energyFromMach = new FXCurve("mach", 1f);

    public FXCurve speedFromMach = new FXCurve("mach", 1f);

    public FXCurve growFromMach = new FXCurve("mach", 0f);

    public FXCurve scaleFromMach = new FXCurve("mach", 1f);

    public FXCurve sizeFromMach = new FXCurve("mach", 1f);

    public FXCurve offsetFromMach = new FXCurve("mach", 0f);


    // Those 2 curve are related to the angle and distance to cam
    public FXCurve angle = new FXCurve("angle", 1f);
    public FXCurve distance = new FXCurve("distance", 1f);

    private List<PersistantKSPParticleEmitter> peristantEmitters;

    private float emissionPower;
    private float minEmissionBase;
    private float maxEmissionBase;

    private float energyPower;
    private float minEnergyBase;
    private float maxEnergyBase;


    private float sizePower;
    private float minSizeBase;
    private float maxSizeBase;

    private float currentScale;
    private float scale1DBase;
    private Vector2 scale2DBase;
    private Vector3 scale3DBase;


    private float localVelocityPower;
    private Vector3 localVelocityBase;

    private Shader shader;



    public static int activeParticles = 0;
    public static int particuleDecimate = 0;
    public static int particleCounter = 0;

    // Particule Emitter with more than decimateFloor particules will have
    // some particle culled if there is more than maximumActiveParticles active
    public static int decimateFloor = 30;
    public static int maximumActiveParticles = 20000;

    private void OnDestroy()
    {
        if (peristantEmitters == null)
        {
            return;
        }
        for (int i = 0; i < peristantEmitters.Count; i++)
            if (peristantEmitters[i].go != null && peristantEmitters[i].go.transform.parent != null)
            {
              peristantEmitters[i].fixedEmit = false; 
              peristantEmitters[i].pe.emit = false; 

                // detach from the parent so the emmitter(and its particle) don't get removed instantly
                peristantEmitters[i].go.transform.parent = null;
            }
    }

    public override void OnEvent()
    {
        if (peristantEmitters == null)
        {
            return;
        }

        UpdateEmitters(1);
        for (int i = 0; i < peristantEmitters.Count; i++)
            peristantEmitters[i].pe.Emit();
    }

    public override void OnEvent(float power)
    {
        if (peristantEmitters == null)
            return;

        if (power > 0f)
        {
            UpdateEmitters(power);
            for (int i = 0; i < peristantEmitters.Count; i++) {
              if (fixedEmissions) {
                peristantEmitters[i].fixedEmit = true;
                peristantEmitters[i].pe.emit = false;
              } else {
                peristantEmitters[i].pe.emit = true;
              }
            }
        }
        else
        {
          for (int j = 0; j < peristantEmitters.Count; j++) {
              peristantEmitters[j].fixedEmit = false;
              peristantEmitters[j].pe.emit = false;
          }
        }
    }

    private List<Component> partColliders = new List<Component>();

    bool addedLaunchPadCollider = false;

    public void FixedUpdate()
    {
        if (peristantEmitters == null)
        {
            return;
        }

        ResetParticleCount();


        RaycastHit hit = new RaycastHit();
        bool collision = (collide != "false");

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

        for (int i = 0; i < peristantEmitters.Count; i++)
        {
          // Emit particles on fixedUpdate rather than Update so that we know which particles
          // were just created and should be nudged, should not be collided, etc.
          if(peristantEmitters[i].fixedEmit) {
            // Number of particles to emit:
            double averageEmittedParticles = UnityEngine.Random.Range(peristantEmitters[i].pe.minEmission, peristantEmitters[i].pe.maxEmission) * TimeWarp.fixedDeltaTime;
            int emittedParticles = (int)Math.Floor(averageEmittedParticles) + (UnityEngine.Random.value < averageEmittedParticles - Math.Floor(averageEmittedParticles) ? 1 : 0);
            for (int k = 0; k < emittedParticles; ++k) {
              peristantEmitters[i].pe.EmitParticle();
            }
          }
            Particle[] particles = peristantEmitters[i].pe.pe.particles;

            for (int j = 0; j < particles.Length; j++)
            {   
                // Check if we need to cull the number of particles

                if (particuleDecimate != 0 && particles.Length > decimateFloor)
                {
                    particleCounter++;
                    if ((particuleDecimate > 0 && (particleCounter % particuleDecimate) == 0) || (particuleDecimate < 0 && (particleCounter % particuleDecimate) != 0))
                        particles[j].energy = 0; // energy set to 0 remove the particle, as per Unity doc
                }

                if (particles[j].energy > 0)
                {
                  Vector3d pPos = peristantEmitters[i].pe.useWorldSpace ? particles[j].position : peristantEmitters[i].pe.transform.TransformPoint(particles[j].position);
                  Vector3d pVel = peristantEmitters[i].pe.useWorldSpace ? particles[j].velocity : peristantEmitters[i].pe.transform.TransformDirection(particles[j].velocity);

                    particles[j].size = Mathf.Min(particles[j].size, sizeClamp);
                    // No need to waste time doing a division if the result is 0.
                    if(logarithmicGrowth != 0.0) {
                      // Euler integration of the derivative of Log(logarithmicGrowth * t + 1) + 1.
                      // TODO(robin): We use minSize rather than keeping the initial size.
                      // This might look weird.
                      particles[j].size += (float)(((TimeWarp.fixedDeltaTime * logarithmicGrowth) / (1 + (particles[j].startEnergy - particles[j].energy) * logarithmicGrowth)) * peristantEmitters[i].pe.minSize);
                    }

                    if (fixedEmissions && particles[j].energy == particles[j].startEnergy) {
                      // Uniformly scatter the particles along the emitter's trajectory in order to remove the dotted smoke effect.
                      pPos -= hostPart.rb.velocity * UnityEngine.Random.value * variableDeltaTime;
                    }

                    if (physical) {
                      double r = particles[j].size;
                      double rMin = peristantEmitters[i].pe.minSize;
                      // TODO(robin): using rMin is probably a bad idea, as above.
                      // There must be a way to keep the actual initial volume, 
                      // but I'm lazy.
                      // N.B.: multiplications rather than Pow, Pow is slow,
                      // multiplication by .5 rather than division by 2 (same 
                      // reason).
                      double estimatedInitialVolume = 0.75 * Math.PI * rMin * rMin * rMin;
                      double currentVolume = 0.75 * Math.PI * r * r * r;
                      double volumeChange = currentVolume - estimatedInitialVolume;
                      double atmosphericDensity = FlightGlobals.getAtmDensity(FlightGlobals.getStaticPressure(pPos));
                      double density = (estimatedInitialVolume * initialDensity + volumeChange * atmosphericDensity) / currentVolume;
                      double mass = density * currentVolume;
                      // Weight and buoyancy.
                      Vector3d acceleration = (1 - (atmosphericDensity / density)) * FlightGlobals.getGeeForceAtPosition(pPos);
                      // Drag. TODO(robin): simplify.
                      acceleration += - 0.5 * atmosphericDensity * pVel * pVel.magnitude * dragCoefficient * Math.PI * r * r / mass;
                      // Euler is good enough for graphics.
                      pVel = pVel + acceleration * TimeWarp.fixedDeltaTime;
                      particles[j].velocity = (peristantEmitters[i].pe.useWorldSpace ? (Vector3)pVel : peristantEmitters[i].pe.transform.InverseTransformDirection(pVel));
                    }

                    if (particles[j].energy != particles[j].startEnergy && // Do not collide newly created particles (they collide with the emitter and things look bad).
                        collision)
                    {
                        if (Physics.Raycast(pPos, pVel, out hit, particles[j].velocity.magnitude * 2f * TimeWarp.fixedDeltaTime, mask))
                            
                            if (hit.collider.name != "Launch Pad Grate")
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
                  
                  particles[j].velocity = (peristantEmitters[i].pe.useWorldSpace ? (Vector3)pVel : peristantEmitters[i].pe.transform.InverseTransformDirection(pVel));
                  particles[j].position = (peristantEmitters[i].pe.useWorldSpace ? (Vector3)pPos : peristantEmitters[i].pe.transform.InverseTransformPoint(pPos));
                }
            }
            peristantEmitters[i].pe.pe.particles = particles;
            activeParticles += peristantEmitters[i].pe.pe.particleCount;

        }
    }

    private const string LaunchPadColliderName = "LaunchPadColliderSmokeScreen";

    private void AddLaunchPadColliders(RaycastHit hit)
    {
        // the Grate Collider size is  (37.70, 20.22, 3.47). Way larger that the actual grate
        // The current collider do not cover all this area. More are needed

        Transform parentTransform = hit.collider.gameObject.transform;

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
    private void ResetParticleCount()
    {
        if (lastTime != Time.fixedTime)
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


    public void UpdateEmitters(float power)
    {
        float atmDensity = 1;
        float surfaceVelMach = 1;
        if (hostPart != null && hostPart.vessel != null)
        {
            Vessel vessel = hostPart.vessel;
            atmDensity = (float)vessel.atmDensity;

            // does not work
            //float p = (float)FlightGlobals.getStaticPressure(vessel.altitude, vessel.mainBody);
            //float speedOfSound = Mathf.Sqrt(1.4f * p / atmDensity);

            // Cheating for now
            double speedOfSound = 343.2f;

            surfaceVelMach = (float)(vessel.srf_velocity.magnitude / speedOfSound);
        }

        for (int i = 0; i < peristantEmitters.Count; i++)
        {
            sizePower = size.Value(power) * sizeFromDensity.Value(atmDensity) * sizeFromMach.Value(surfaceVelMach);
            peristantEmitters[i].pe.minSize = Mathf.Min(minSizeBase * sizePower, sizeClamp);
            peristantEmitters[i].pe.maxSize = Mathf.Min(maxSizeBase * sizePower, sizeClamp);

            emissionPower = emission.Value(power) * emissionFromDensity.Value(atmDensity) * emissionFromMach.Value(surfaceVelMach);
            peristantEmitters[i].pe.minEmission = Mathf.FloorToInt(minEmissionBase * emissionPower);
            peristantEmitters[i].pe.maxEmission = Mathf.FloorToInt(maxEmissionBase * emissionPower);
            
            energyPower = energy.Value(power) * energyFromDensity.Value(atmDensity) * energyFromMach.Value(surfaceVelMach);
            peristantEmitters[i].pe.minEnergy = minEnergyBase * energyPower;
            peristantEmitters[i].pe.maxEnergy = maxEnergyBase * energyPower;
            
            localVelocityPower = speed.Value(power) * speedFromDensity.Value(atmDensity) * speedFromMach.Value(surfaceVelMach);
            peristantEmitters[i].pe.localVelocity = localVelocityBase * localVelocityPower;
            
            peristantEmitters[i].pe.sizeGrow = grow.Value(power) + growFromDensity.Value(atmDensity) + growFromMach.Value(surfaceVelMach);

            currentScale = scale.Value(power) * scaleFromDensity.Value(atmDensity) * scaleFromMach.Value(surfaceVelMach);
            peristantEmitters[i].pe.shape1D = scale1DBase * currentScale;
            peristantEmitters[i].pe.shape2D = scale2DBase * currentScale;
            peristantEmitters[i].pe.shape3D = scale3DBase * currentScale;


            peristantEmitters[i].go.transform.localPosition = Vector3d.forward * ( offset.Value(power) + offsetFromDensity.Value(atmDensity) + offsetFromMach.Value(surfaceVelMach));

            //print(atmDensity.ToString("F2") + " " + offset.Value(power).ToString("F2") + " " + offsetFromDensity.Value(atmDensity).ToString("F2") + " " + offsetFromMach.Value(surfaceVelMach).ToString("F2"));
        }
    }

    public void Update()
    {
      variableDeltaTime = Time.deltaTime;
        if (peristantEmitters == null)
        {
            return;
        }
        for (int i = 0; i < peristantEmitters.Count; i++)
        {
            // using Camera.main will mess up anything multi cam but using current require adding a OnWillRenderObject() to the ksp particle emitter GameObject (? not tested)
            float currentAngle = Vector3.Angle(-Camera.main.transform.forward, peristantEmitters[i].go.transform.forward);
            float currentDist = (Camera.main.transform.position - peristantEmitters[i].go.transform.position).magnitude;

            peristantEmitters[i].pe.maxParticleSize = peristantEmitters[i].baseMaxSize * angle.Value(currentAngle) * distance.Value(currentDist);
            peristantEmitters[i].pe.pr.maxParticleSize = peristantEmitters[i].pe.maxParticleSize;
        }
    }


    public override void OnInitialize()
    {
        // The shader loading require proper testing
        // Unity doc says that "Creating materials this way supports only simple shaders (fixed function ones). 
        // If you need a surface shader, or vertex/pixel shaders, you'll need to create shader asset in the editor and use that."
        // But importing the same shader that the one used in the editor seems to work
        string filename = KSPUtil.ApplicationRootPath + "GameData/" + shaderFileName;
        if (shaderFileName != String.Empty && System.IO.File.Exists(filename))
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
            templateKspParticleEmitter.material.shader = shader;


        // TODO : move those in PersistantKSPParticleEmitter 
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

        if (peristantEmitters == null)
            peristantEmitters = new List<PersistantKSPParticleEmitter>();


        for (int i = 0; i < transforms.Count; i++)
        {

            GameObject emmitterGameObject = UnityEngine.Object.Instantiate(model) as GameObject;
            KSPParticleEmitter componentInChildren = emmitterGameObject.GetComponentInChildren<KSPParticleEmitter>();

            PersistantKSPParticleEmitter pkpe = new PersistantKSPParticleEmitter(emmitterGameObject, componentInChildren, templateKspParticleEmitter.maxParticleSize);

            if (componentInChildren != null)
            {
                componentInChildren.shape1D *= fixedScale;
                componentInChildren.shape2D *= fixedScale;
                componentInChildren.shape3D *= fixedScale;

                try
                {
                    componentInChildren.particleRenderMode = (ParticleRenderMode)Enum.Parse(typeof(ParticleRenderMode), renderMode);
                }
                catch (ArgumentException)
                {
                    print("ModelMultiParticleFXExt: " + renderMode + " is not a valid ParticleRenderMode");
                }


                peristantEmitters.Add(pkpe);

            }

            emmitterGameObject.transform.SetParent(transforms[i]);

            emmitterGameObject.transform.localPosition = localPosition;
            emmitterGameObject.transform.localRotation = Quaternion.Euler(localRotation);

            PersistantEmitterManager.Add(pkpe);

        }


        UnityEngine.Object.Destroy(templateKspParticleEmitter);
    }

    public override void OnLoad(ConfigNode node)
    {
        ConfigNode.LoadObjectFromConfig(this, node);
        emission.Load("emission", node);
        energy.Load("energy", node);
        speed.Load("speed", node);
        grow.Load("grow", node);
        scale.Load("scale", node);

        size.Load("size", node);

        offset.Load("offset", node);

        

        angle.Load("angle", node);
        distance.Load("distance", node);


        if (node.HasNode("emission"))
        {
            emissionFromDensity.Load("density", node.GetNode("emission"));
            emissionFromMach.Load("mach", node.GetNode("emission"));
        }

        if (node.HasNode("energy"))
        {
            energyFromDensity.Load("density", node.GetNode("energy"));
            energyFromMach.Load("mach", node.GetNode("energy"));
        }

        if (node.HasNode("speed"))
        {
            speedFromDensity.Load("density", node.GetNode("speed"));
            speedFromMach.Load("mach", node.GetNode("speed"));
        }

        if (node.HasNode("grow"))
        {
            growFromDensity.Load("density", node.GetNode("grow"));
            growFromMach.Load("mach", node.GetNode("grow"));
        }

        if (node.HasNode("size"))
        {
            sizeFromDensity.Load("density", node.GetNode("size"));
            sizeFromMach.Load("mach", node.GetNode("size"));
        }

        if (node.HasNode("scale"))
        {
            scaleFromDensity.Load("density", node.GetNode("scale"));
            scaleFromMach.Load("mach", node.GetNode("scale"));
        }

        if (node.HasNode("offset"))
        {
            offsetFromDensity.Load("density", node.GetNode("offset"));
            offsetFromMach.Load("mach", node.GetNode("offset"));
        }


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

        ConfigNode subNode = new ConfigNode("emission");
        emissionFromDensity.Save(subNode);
        emissionFromMach.Save(subNode);
        node.AddNode(subNode);

        subNode = new ConfigNode("energy");
        energyFromDensity.Save(subNode);
        energyFromMach.Save(subNode);
        node.AddNode(subNode);

        subNode = new ConfigNode("speed");
        speedFromDensity.Save(subNode);
        speedFromMach.Save(subNode);
        node.AddNode(subNode);

        subNode = new ConfigNode("grow");
        growFromDensity.Save(subNode);
        growFromMach.Save(subNode);
        node.AddNode(subNode);

        subNode = new ConfigNode("size");
        sizeFromDensity.Save(subNode);
        sizeFromMach.Save(subNode);
        node.AddNode(subNode);


        subNode = new ConfigNode("scale");
        scaleFromDensity.Save(subNode);
        scaleFromMach.Save(subNode);
        node.AddNode(subNode);


        subNode = new ConfigNode("emoffset");
        offsetFromDensity.Save(subNode);
        offsetFromMach.Save(subNode);
        node.AddNode(subNode);
    }

    private void print(String s)
    {
        MonoBehaviour.print(this.GetType().Name + " : " + s);
    }

}
