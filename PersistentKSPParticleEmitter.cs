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

using SmokeScreen;

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

    public float linearGrow;

    public float sizeClamp = 50;

    // The initial velocity of the particles will be offset by a random amount
    // lying in a disk perpendicular to the mean initial velocity whose radius
    // is randomOffsetMaxRadius. This is similar to Unity's 'Random Velocity'
    // Setting, except it will sample the offset from a (normal) disk rather
    // than from a cube. Units (SI): m/s.
    public float randomInitalVelocityOffsetMaxRadius = 0.0f;

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

        this.scale1DBase = templateKspParticleEmitter.shape1D;
        this.scale2DBase = templateKspParticleEmitter.shape2D;
        this.scale3DBase = templateKspParticleEmitter.shape3D;

        this.minEmissionBase = (float)templateKspParticleEmitter.minEmission;
        this.maxEmissionBase = (float)templateKspParticleEmitter.maxEmission;
        this.minEnergyBase = templateKspParticleEmitter.minEnergy;
        this.maxEnergyBase = templateKspParticleEmitter.maxEnergy;

        this.minSizeBase = (float)templateKspParticleEmitter.minSize;
        this.maxSizeBase = (float)templateKspParticleEmitter.maxSize;

        this.localVelocityBase = templateKspParticleEmitter.localVelocity;
        this.worldVelocityBase = templateKspParticleEmitter.worldVelocity;

        this.forceBase = templateKspParticleEmitter.force;

        PersistentEmitterManager.Add(this);
    }

    // Detach the emitter from its parent gameobject and stop its emmission in timer seconds
    public void Detach(float timer)
    {
        this.endTime = Time.fixedTime + timer;
        if (this.go != null && this.go.transform.parent != null)
        {
            // detach from the parent so the emmitter(and its particle) don't get removed instantly
            this.go.transform.parent = null;
        }
    }

    public void EmissionStop()
    {
        this.fixedEmit = false;
        this.pe.emit = false;
    }

    // Update the particles of the Emitter : Emit, resize, collision and physic
    public void EmitterOnUpdate(Vector3 emitterWorldVelocity)
    {
        RaycastHit hit = new RaycastHit();
        // "Default", "TransparentFX", "Local Scenery", "Ignore Raycast"
        int mask = (1 << LayerMask.NameToLayer("Default")) | (1 << LayerMask.NameToLayer("Local Scenery"));

        // Emit particles on fixedUpdate rather than Update so that we know which particles
        // were just created and should be nudged, should not be collided, etc.
        if (this.fixedEmit)
        {
            // Number of particles to emit:
            double averageEmittedParticles = Random.Range(this.pe.minEmission, this.pe.maxEmission)
                                             * TimeWarp.fixedDeltaTime;
            double compensatedEmittedParticles = averageEmittedParticles + this.particleFraction;
            double emittedParticles = Math.Truncate(compensatedEmittedParticles);
            this.particleFraction = compensatedEmittedParticles - emittedParticles;

            int emissionCount = (int)emittedParticles;
            for (int k = 0; k < emissionCount; ++k)
            {
                this.pe.EmitParticle();
            }
        }

        // This line (and the one that does the oposite at the end) is actally the slowest part of the whole function 
        Particle[] particles = this.pe.pe.particles;

        double averageSize = 0.5 * (this.pe.minSize + this.pe.maxSize);

        Particle particle;
        for (int j = 0; j < particles.Length; j++)
        {
            particle = particles[j];
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
                Vector3d pPos = this.pe.useWorldSpace
                                    ? particle.position
                                    : this.pe.transform.TransformPoint(particle.position);
                Vector3d pVel = (this.pe.useWorldSpace
                                     ? particle.velocity
                                     : this.pe.transform.TransformDirection(particle.velocity))
                                + Krakensbane.GetFrameVelocity();

                // try-finally block to ensure we set the particle velocities correctly in the end.
                try
                {
                    // Fixed update is not the best place to update the size but the particles array copy is 
                    // slow so doing each frame would be worse

                    // No need to waste time doing a division if the result is 0.
                    if (this.logarithmicGrow != 0.0)
                    {
                        // Euler integration of the derivative of Log(logarithmicGrowth * t + 1) + 1.
                        // This might look weird.
                        particle.size +=
                            (float)
                            (((TimeWarp.fixedDeltaTime * this.logarithmicGrow)
                              / (1 + (particle.startEnergy - particle.energy) * this.logarithmicGrow)) * averageSize);
                    }
                    if (this.linearGrow != 0.0)
                    {
                        particle.size += (float)(TimeWarp.fixedDeltaTime * this.linearGrow * averageSize);
                    }

                    particle.size = Mathf.Min(particle.size, this.sizeClamp);

                    if (particle.energy == particle.startEnergy)
                    {
                        if (this.pe.useWorldSpace)
                        {
                            // Uniformly scatter newly emitted particles along the emitter's trajectory in order to 
                            // remove the dotted smoke effect.
                            // use variableDeltaTime since the particle are emited on Update anyway.
                            pPos -= emitterWorldVelocity * Random.value * Time.deltaTime;
                        }
                        if (this.randomInitalVelocityOffsetMaxRadius != 0.0)
                        {
                            Vector2 diskPoint = Random.insideUnitCircle * this.randomInitalVelocityOffsetMaxRadius;
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

                    if (this.physical && (j % physicsPass == activePhysicsPass))
                    {
                        // There must be a way to keep the actual initial volume, 
                        // but I'm lazy.
                        pVel = this.ParticlePhysics(particle.size, averageSize, pPos, pVel);
                    }

                    if (this.collide && particle.energy != particle.startEnergy
                        // Do not collide newly created particles (they collide with the emitter and things look bad).
                        && (j % physicsPass == activePhysicsPass))
                    {
                        pVel = this.ParticleCollision(pPos, pVel, hit, mask);
                    }
                }
                finally
                {
                    particle.velocity = (this.pe.useWorldSpace
                                             ? (Vector3)(pVel - Krakensbane.GetFrameVelocity())
                                             : this.pe.transform.InverseTransformDirection(
                                                 pVel - Krakensbane.GetFrameVelocity()));
                    particle.position = this.pe.useWorldSpace
                                            ? (Vector3)pPos
                                            : this.pe.transform.InverseTransformPoint(pPos);
                }
            }
            particles[j] = particle;
        }
        activePhysicsPass = ++activePhysicsPass % physicsPass;
        this.pe.pe.particles = particles;
        SmokeScreenConfig.activeParticles += this.pe.pe.particleCount;
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

            if (hit.collider.name != SmokeScreenUtil.LaunchPadGrateColliderName)
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
                float randomAngle = Random.value * 360.0f;
                Vector3d outflow = Quaternion.AngleAxis(randomAngle, hit.normal) * unitTangent * residualFlow;
                pVel = hVel + collideRatio * reflectedNormalVelocity + outflow * (1 - stickiness);
            }
            else
            {
                // Don't collide with the launch pad grid and add colliders under it
                if (!this.addedLaunchPadCollider)
                {
                    this.addedLaunchPadCollider = SmokeScreenUtil.AddLaunchPadColliders(hit);
                }
            }
        }
        return pVel;
    }

    private void Print(string s)
    {
        MonoBehaviour.print(this.GetType().Name + " : " + s);
    }
}