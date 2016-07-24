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
using UnityEngine;
using Random = UnityEngine.Random;

// TODO : handle the relation with PersistentEmitterManager inside the class
public class PersistentKSPParticleEmitter
{
    public GameObject go;

    public KSPParticleEmitter pe;

    public bool fixedEmit = false;

    public float endTime = 0;

    public double particleFraction = 0;

    public readonly float minEmissionBase;

    public readonly float maxEmissionBase;

    public readonly float minEnergyBase;

    public readonly float maxEnergyBase;

    public readonly float minSizeBase;

    public readonly float maxSizeBase;

    public readonly float scale1DBase;

    public readonly Vector2 scale2DBase;

    public readonly Vector3 scale3DBase;

    public readonly Vector3 localVelocityBase;

    public readonly Vector3 worldVelocityBase;

    public readonly Vector3 forceBase;

    public float logarithmicGrow;

    public float logarithmicGrowScale;

    public float linearGrow;

    public float sizeClamp = 50;

    public bool clampXYstart = false;

    // The initial velocity of the particles will be offset by a random amount
    // lying in a disk perpendicular to the mean initial velocity whose radius
    // is randomOffsetMaxRadius. This is similar to Unity's 'Random Velocity'
    // Setting, except it will sample the offset from a (normal) disk rather
    // than from a cube. Units (SI): m/s.
    public float randomInitalVelocityOffsetMaxRadius = 0.0f;

    //Similar to randomInitalVelocityOffsetMaxRadius, cleaned a little 
    public float randConeEmit = 0.0f;

    //Additive random position offset
    public float vRandPosOffset = 0.0f;

    //Additive non-random position offset
    public float vPosOffset = 0.0f;

    //xyForce multiplicatively damps non-axis (x,y) motion, leaving axis 
    //motion (z) untouched.
    public float xyForce = 1.0f;

    //zForce is like xyForce, but directly adjusts in the direction of motion,
    public float zForce = 1.0f;

    // Whether to apply Archimedes' force, gravity and other things to the
    // particle.
    public bool physical = false;

    // Initial density of the particle seen as sphere of radius size of perfect
    // gas. We then assume (only true for ideally expanded exhaust) that the
    // expansion is isobaric (by mixing with the atmosphere) in order to compute
    // the density afterwards. Units (SI): kg / m^3.
    public double initialDensity = .6;

    public double dragCoefficient = 0.1;

    // How much the particles stick to objects they collide with.
    public double stickiness = 0.9;

    public bool collide = false;

    public float collideRatio = 0.0f;

    private bool addedLaunchPadCollider = false;

    private static uint physicsPass = 4;

    private static uint activePhysicsPass = 0;

    public PersistentKSPParticleEmitter(
        GameObject go,
        KSPParticleEmitter pe,
        KSPParticleEmitter templateKspParticleEmitter)
    {
        this.go = go;
        this.pe = pe;

        scale1DBase = templateKspParticleEmitter.shape1D;
        scale2DBase = templateKspParticleEmitter.shape2D;
        scale3DBase = templateKspParticleEmitter.shape3D;

        minEmissionBase = templateKspParticleEmitter.minEmission;
        maxEmissionBase = templateKspParticleEmitter.maxEmission;
        minEnergyBase = templateKspParticleEmitter.minEnergy;
        maxEnergyBase = templateKspParticleEmitter.maxEnergy;

        minSizeBase = templateKspParticleEmitter.minSize;
        maxSizeBase = templateKspParticleEmitter.maxSize;

        localVelocityBase = templateKspParticleEmitter.localVelocity;
        worldVelocityBase = templateKspParticleEmitter.worldVelocity;

        forceBase = templateKspParticleEmitter.force;

        PersistentEmitterManager.Add(this);
    }

    // Detach the emitter from its parent gameobject and stop its emmission in timer seconds
    public void Detach(float timer)
    {
        //Print("Detach");
        endTime = Time.fixedTime + timer;
        if (go != null && go.transform.parent != null)
        {
            // detach from the parent so the emmitter(and its particle) don't get removed instantly
            go.transform.parent = null;
        }
    }

    public void EmissionStop()
    {
        fixedEmit = false;
        pe.emit = false;
    }

    // Update the particles of the Emitter : Emit, resize, collision and physic
    public void EmitterOnUpdate(Vector3 emitterWorldVelocity)
    {
        if (pe == null || pe.pe == null)
            return;


        // "Default", "TransparentFX", "Local Scenery", "Ignore Raycast"
        int mask = (1 << LayerMask.NameToLayer("Default")) | (1 << LayerMask.NameToLayer("Local Scenery"));

        // Emit particles on fixedUpdate rather than Update so that we know which particles
        // were just created and should be nudged, should not be collided, etc.
        if (fixedEmit)
        {
            // Number of particles to emit:
            double averageEmittedParticles = Random.Range(pe.minEmission, pe.maxEmission)
                                             * TimeWarp.fixedDeltaTime;
            double compensatedEmittedParticles = averageEmittedParticles + particleFraction;
            double emittedParticles = Math.Truncate(compensatedEmittedParticles);
            particleFraction = compensatedEmittedParticles - emittedParticles;

            int emissionCount = (int)emittedParticles;
            for (int k = 0; k < emissionCount; ++k)
            {
                pe.EmitParticle();
            }
        }

        // This line (and the one that does the oposite at the end) is actally the slowest part of the whole function
        Particle[] particles = pe.pe.particles;

        double averageSize = 0.5 * (pe.minSize + pe.maxSize);
        
        //For randConeEmit:
        //Only generate a random vector on every other particle, for the in-between particles, negate the disk.
        bool toggle = true;
        Vector2 disk = new Vector2 (0,0);
        //For startSpread

        double logGrowConst = TimeWarp.fixedDeltaTime * logarithmicGrow * logarithmicGrowScale;
        float linGrowConst = (float)(TimeWarp.fixedDeltaTime * linearGrow * averageSize);

        Transform peTransform = pe.transform;

        Vector3d frameVel = Krakensbane.GetFrameVelocity();

        //Step through all the particles:
        for (int j = 0; j < particles.Length; j++)
        {
            Particle particle = particles[j];

            // Check if we need to cull the number of particles
            if (SmokeScreenConfig.particleDecimate != 0 && particles.Length > SmokeScreenConfig.decimateFloor)
            {
                SmokeScreenConfig.particleCounter++;
                if ((SmokeScreenConfig.particleDecimate > 0
                     && (SmokeScreenConfig.particleCounter % SmokeScreenConfig.particleDecimate) == 0)
                    || (SmokeScreenConfig.particleDecimate < 0
                        && (SmokeScreenConfig.particleCounter % SmokeScreenConfig.particleDecimate) != 0))
                {
                    particle.energy = 0; // energy set to 0 remove the particle, as per Unity doc
                }
            }

            
            if (particle.energy > 0)
            {
                //Slight methodology change to avoid duplicating if statements:
                Vector3d pVel;
                Vector3d pPos;
                if (pe.useWorldSpace)
                {
                    pVel = particle.velocity + frameVel;
                    pPos = particle.position;
                }
                else if (!pe.useWorldSpace && particle.energy == particle.startEnergy)
                {
                    Vector3 lVel = new Vector3(0, 0, 1);
                    Vector3 lPos = particle.position;

                    // Adjust initial velocity to make a cone.  Only perform if pe.useWorldSpace
                    // is true, and we have a randConeEmit set.
                    //Produce a random vector within "angle" of the original vector.
                    //The maximum producible cone is 90 degrees when randConeEmit is very large.
                    //Could open up more if we used trig, but it'd be less efficient.

                    if (toggle)
                    {
                        disk = Random.insideUnitCircle * randConeEmit;
                        toggle = false;
                    }
                    else
                    {
                        disk *= -1;
                        toggle = true;
                    }
                    lVel.x = disk.x;
                    lVel.y = disk.y;
                    lVel = Vector3.Normalize(lVel);
                    lVel *= Vector3.Magnitude(particle.velocity);

                    //Adjust initial position back along its position, if required.
                    //Apply a random offset if vRandOffset != 0, else apply zero.
                    float randoff = (vRandPosOffset != 0)? Random.Range(0, vRandPosOffset) : 0;
                    lPos += Vector3.Normalize(lVel) * (randoff + vPosOffset);

                    //Finalize position and velocity
                    pPos = peTransform.TransformPoint(lPos);
                    pVel = peTransform.TransformDirection(lVel)
                                + frameVel;
                }
                else if (!pe.useWorldSpace && particle.energy != particle.startEnergy)
                {
                    pPos = peTransform.TransformPoint(particle.position);
                    pVel = peTransform.TransformDirection(particle.velocity.x * xyForce,
                                                           particle.velocity.y * xyForce,
                                                           particle.velocity.z * zForce)
                                + frameVel;
                }
                else
                {
                    pPos = peTransform.TransformPoint(particle.position);
                    pVel = peTransform.TransformDirection(particle.velocity) + frameVel;
                }
                
                // try-finally block to ensure we set the particle velocities correctly in the end.
                try
                {
                    // Fixed update is not the best place to update the size but the particles array copy is
                    // slow so doing each frame would be worse
                    
                    // No need to waste time doing a division if the result is 0.
                    if (logarithmicGrow != 0.0)
                    {
                        // Euler integration of the derivative of Log(logarithmicGrowth * t + 1) + 1.
                        // This might look weird.
                        
                        particle.size += (float) ((logGrowConst / (1 + (particle.startEnergy - particle.energy) * logarithmicGrow)) * averageSize);
                    }
                    if (linearGrow != 0.0)
                    {
                        particle.size += linGrowConst;
                    }
                    
                    particle.size = Mathf.Min(particle.size, sizeClamp);

                    if (particle.energy == particle.startEnergy)
                    {
                        
                        if (pe.useWorldSpace)
                        {
                            // Uniformly scatter newly emitted particles along the emitter's trajectory in order to
                            // remove the dotted smoke effect.
                            // use variableDeltaTime since the particle are emited on Update anyway.
                            pPos -= emitterWorldVelocity * Random.value * Time.deltaTime;
                        }

                        if (randomInitalVelocityOffsetMaxRadius != 0.0)
                        {
                            Vector2 diskPoint = Random.insideUnitCircle * randomInitalVelocityOffsetMaxRadius;
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

                    if (physical && (j % physicsPass == activePhysicsPass))
                    {
                        // There must be a way to keep the actual initial volume,
                        // but I'm lazy.
                        pVel = ParticlePhysics(particle.size, averageSize, pPos, pVel);
                    }

                    if (collide && particle.energy != particle.startEnergy

                        // Do not collide newly created particles (they collide with the emitter and things look bad).
                        && (j % physicsPass == activePhysicsPass))
                    {
                        pVel = ParticleCollision(pPos, pVel, mask);
                    }

                }
                finally
                {
                    particle.velocity = (pe.useWorldSpace
                        ? (Vector3)(pVel - frameVel)
                        : peTransform.InverseTransformDirection(pVel - frameVel));
                    particle.position = pe.useWorldSpace
                        ? (Vector3)pPos
                        : peTransform.InverseTransformPoint(pPos);
                }
            }
            particles[j] = particle;
        }
        activePhysicsPass = ++activePhysicsPass % physicsPass;
        pe.pe.particles = particles;
        SmokeScreenConfig.activeParticles += pe.pe.particleCount;
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
        double atmosphericDensity = FlightGlobals.getAtmDensity(FlightGlobals.getStaticPressure(pPos, mainBody), FlightGlobals.getExternalTemperature(pPos, FlightGlobals.currentMainBody), FlightGlobals.currentMainBody);
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
        return pVel + acceleration * TimeWarp.fixedDeltaTime * physicsPass;
    }

    private Vector3 ParticleCollision(Vector3d pPos, Vector3d pVel, int mask)
    {
        RaycastHit hit;
        if (Physics.Raycast(
            pPos,
            pVel,
            out hit,
            (float)pVel.magnitude * TimeWarp.fixedDeltaTime * physicsPass,
            mask))
        {
            //// collidersName[hit.collider.name] = true;

            if (hit.collider.name != SmokeScreenUtil.LaunchPadGrateColliderName)
            {
                Vector3 unitTangent = (hit.normal.x == 0 && hit.normal.y == 0)
                    ? new Vector3(1, 0, 0)
                    : Vector3.ProjectOnPlane(new Vector3(0, 0, 1), hit.normal).normalized;
                 Vector3 hVel = Vector3.ProjectOnPlane(pVel, hit.normal);
                Vector3 reflectedNormalVelocity = hVel - pVel;
                float residualFlow = reflectedNormalVelocity.magnitude * (1 - collideRatio);

                // An attempt at a better velocity change; the blob collides with some
                // restitution coefficient collideRatio << 1 and we add a random tangential term
                // for outflowing particles---randomness handwaved in through fluid dynamics:
                float randomAngle = Random.value * 360.0f;
                Vector3d outflow = Quaternion.AngleAxis(randomAngle, hit.normal) * unitTangent * residualFlow;
                pVel = hVel + collideRatio * reflectedNormalVelocity + outflow * (1 - stickiness);
            }
            else
            {
                // Don't collide with the launch pad grid and add colliders under it
                if (!addedLaunchPadCollider)
                {
                    addedLaunchPadCollider = SmokeScreenUtil.AddLaunchPadColliders(hit);
                }
            }
        }
        return pVel;
    }
    private Vector3 RandomConeVector(float angle)
    {
        //Performed in the transform's frame, default output should be 0,0,1
        Vector3 unit = new Vector3(0,0,1);
        //Produce a random vector within "angle" of the original vector.
        Vector2 disk = Random.insideUnitCircle*angle;
        unit.x = disk.x;
        unit.y = disk.y;
        unit = Vector3.Normalize(unit);
        return unit;
    }
    private void Print(string s)
    {
        MonoBehaviour.print("[SmokeScreen " + GetType().Name + "] : " + s);
    }
}