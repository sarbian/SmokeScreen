/*
 * Author: Sébastien GAGGINI AKA Sarbian, France
 * License: BY: Attribution 4.0 International (CC BY 4.0): http://creativecommons.org/licenses/by/4.0/
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
    public Vector3 localRotation = Vector3.zero;

    [Persistent]
    public Vector3 localPosition = Vector3.zero;

    [Persistent]
    public float fixedScale = 1;

    [Persistent]
    public float sizeClamp = 50;

    [Persistent]
    public double maxAngle = 181;

    [Persistent]
    public float minDist = 0;

    public FXCurve emission = new FXCurve("emission", 1f);

    public FXCurve energy = new FXCurve("energy", 1f);

    public FXCurve speed = new FXCurve("speed", 1f);

    public FXCurve grow = new FXCurve("grow", 0f);

    public FXCurve scale = new FXCurve("scale", 1f);


    public FXCurve emissionFromDensity = new FXCurve("density", 1f);

    public FXCurve energyFromDensity = new FXCurve("density", 1f);

    public FXCurve speedFromDensity = new FXCurve("density", 1f);

    public FXCurve growFromDensity = new FXCurve("density", 1f);

    public FXCurve scaleFromDensity = new FXCurve("density", 1f);

    public FXCurve emissionFromMach = new FXCurve("mach", 1f);

    public FXCurve energyFromMach = new FXCurve("mach", 1f);

    public FXCurve speedFromMach = new FXCurve("mach", 1f);

    public FXCurve growFromMach = new FXCurve("mach", 1f);

    public FXCurve scaleFromMach = new FXCurve("mach", 1f);


    private List<KSPParticleEmitter> particleEmitters;
    private List<GameObject> emittersGameObjects;

    private float emissionPower;
    private float minEmissionBase;
    private float maxEmissionBase;

    private float energyPower;
    private float minEnergyBase;
    private float maxEnergyBase;

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
    public static int maximumActiveParticles = 10000;

    private void OnDestroy()
    {
        if (particleEmitters == null)
        {
            return;
        }
        for (int i = 0; i < particleEmitters.Count; i++)
            if (emittersGameObjects[i] != null && emittersGameObjects[i].transform.parent != null)
            {
                particleEmitters[i].emit = false;

                // detach from the parent so the emmitter(and its particle) don't get removed instantly
                emittersGameObjects[i].transform.parent = null;
            }
    }

    public override void OnEvent()
    {
        if (particleEmitters == null)
        {
            return;
        }

        UpdateEmitters(1);
        for (int i = 0; i < particleEmitters.Count; i++)
            particleEmitters[i].Emit();
    }

    public override void OnEvent(float power)
    {
        if (particleEmitters == null)
            return;

        if (power > 0f)
        {
            UpdateEmitters(power);
            for (int i = 0; i < particleEmitters.Count; i++)
                particleEmitters[i].emit = true;
        }
        else
        {
            for (int j = 0; j < particleEmitters.Count; j++)
                particleEmitters[j].emit = false;
        }
    }

    private List<Component> partColliders = new List<Component>();

    bool addedLaunchPadCollider = false;

    public void FixedUpdate()
    {
        if (particleEmitters == null)
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

        for (int i = 0; i < particleEmitters.Count; i++)
        {
            Particle[] particles = particleEmitters[i].pe.particles;

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
                    particles[j].size = Mathf.Min(particles[j].size, sizeClamp);

                    if (collision)
                    {
                        Vector3 pPos = particleEmitters[i].pe.useWorldSpace ? particles[j].position : particleEmitters[i].transform.TransformPoint(particles[j].position);
                        Vector3 pVel = particleEmitters[i].pe.useWorldSpace ? particles[j].velocity : particleEmitters[i].transform.TransformDirection(particles[j].velocity);
                        if (Physics.Raycast(pPos, pVel, out hit, particles[j].velocity.magnitude * 3 * TimeWarp.fixedDeltaTime, mask))
                            if (hit.collider.name != "Launch Pad Grate")
                            {
                                Vector3d hVel = Vector3d.Exclude(hit.normal, pVel);
                                Vector3d vVel = Vector3d.Project(pVel, hit.normal);
                                //pVel = (hVel.normalized - 0.05f * vVel.normalized).normalized * pVel.magnitude;
                                pVel = hVel.normalized * pVel.magnitude;
                                particles[j].velocity = particleEmitters[i].pe.useWorldSpace ? pVel : particleEmitters[i].transform.InverseTransformDirection(pVel);
                            }
                            else
                            {
                                // Con't collide with the launch pad grid and add colliders under it
                                if (!addedLaunchPadCollider)
                                    AddLaunchPadColliders(hit);
                            }
                    }
                }
            }
            particleEmitters[i].pe.particles = particles;
            activeParticles += particleEmitters[i].pe.particleCount;

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

            print(activeParticles + " " + particuleDecimate + " " + particleCounter);

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

        for (int i = 0; i < particleEmitters.Count; i++)
        {
            emissionPower = emission.Value(power) * emissionFromDensity.Value(atmDensity) * emissionFromMach.Value(surfaceVelMach);
            particleEmitters[i].minEmission = Mathf.FloorToInt(minEmissionBase * emissionPower);
            particleEmitters[i].maxEmission = Mathf.FloorToInt(maxEmissionBase * emissionPower);
            energyPower = energy.Value(power) * energyFromDensity.Value(atmDensity) * energyFromMach.Value(surfaceVelMach);
            particleEmitters[i].minEnergy = minEnergyBase * energyPower;
            particleEmitters[i].maxEnergy = maxEnergyBase * energyPower;
            localVelocityPower = speed.Value(power) * speedFromDensity.Value(atmDensity) * speedFromMach.Value(surfaceVelMach);
            particleEmitters[i].localVelocity = localVelocityBase * localVelocityPower;
            particleEmitters[i].sizeGrow = grow.Value(power) * growFromDensity.Value(atmDensity) * growFromMach.Value(surfaceVelMach);

            currentScale = scale.Value(power) * scaleFromDensity.Value(atmDensity) * scaleFromMach.Value(surfaceVelMach);
            particleEmitters[i].shape1D = scale1DBase * currentScale;
            particleEmitters[i].shape2D = scale2DBase * currentScale;
            particleEmitters[i].shape3D = scale3DBase * currentScale;


            // using Camera.main will mess up anything multi cam but ussing current require adding a OnWillRenderObject() to the ksp particle emitter GameObject (? not tested)
            double angle = Vector3d.Angle(-Camera.main.transform.forward, emittersGameObjects[i].transform.forward);
            float dist = (Camera.main.transform.position - emittersGameObjects[i].transform.position).magnitude;
            particleEmitters[i].pr.enabled = (angle < maxAngle && dist > minDist);
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

        scale1DBase = (templateKspParticleEmitter.shape1D *= fixedScale);
        scale2DBase = (templateKspParticleEmitter.shape2D *= fixedScale);
        scale3DBase = (templateKspParticleEmitter.shape3D *= fixedScale);

        minEmissionBase = (float)templateKspParticleEmitter.minEmission;
        maxEmissionBase = (float)templateKspParticleEmitter.maxEmission;
        minEnergyBase = templateKspParticleEmitter.minEnergy;
        maxEnergyBase = templateKspParticleEmitter.maxEnergy;
        localVelocityBase = templateKspParticleEmitter.localVelocity;

        if (particleEmitters == null)
            particleEmitters = new List<KSPParticleEmitter>();

        emittersGameObjects = new List<GameObject>();

        for (int i = 0; i < transforms.Count; i++)
        {

            GameObject emmitterGameObject = UnityEngine.Object.Instantiate(model) as GameObject;
            KSPParticleEmitter componentInChildren = emmitterGameObject.GetComponentInChildren<KSPParticleEmitter>();

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
                particleEmitters.Add(componentInChildren);
                emittersGameObjects.Add(emmitterGameObject);
            }

            emmitterGameObject.transform.SetParent(transforms[i]);

            emmitterGameObject.transform.localPosition = localPosition;
            emmitterGameObject.transform.localRotation = Quaternion.Euler(localRotation);

            PersistantEmitterManager.Add(componentInChildren, emmitterGameObject);

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

        if (node.HasNode("scale"))
        {
            scaleFromDensity.Load("density", node.GetNode("scale"));
            scaleFromMach.Load("mach", node.GetNode("scale"));
        }


    }

    public override void OnSave(ConfigNode node)
    {
        ConfigNode.CreateConfigFromObject(this, node);
        emission.Save(node);
        energy.Save(node);
        speed.Save(node);
        grow.Save(node);

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

        subNode = new ConfigNode("scale");
        scaleFromDensity.Save(subNode);
        scaleFromMach.Save(subNode);
        node.AddNode(subNode);
    }

    private void print(String s)
    {
        MonoBehaviour.print(this.GetType().Name + " : " + s);
    }



}
