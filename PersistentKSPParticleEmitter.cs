/*
 * Author: Sébastien GAGGINI AKA Sarbian, France
 * License: Attribution 4.0 International (CC BY 4.0): http://creativecommons.org/licenses/by/4.0/
 * 
 * Thanks to Nothke for all the feature ideas, testing and feedback
 * 
 */


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;


// TODO : handle the relation with PersistentEmitterManager inside the class
public class PersistentKSPParticleEmitter
{
    public GameObject go;
    public KSPParticleEmitter pe;
    public bool fixedEmit = false;

    public float timer = 0;

    public readonly float minEmissionBase;
    public readonly float maxEmissionBase;

    public readonly float minEnergyBase;
    public readonly float maxEnergyBase;

    public readonly float minSizeBase;
    public readonly float maxSizeBase;

    public readonly float scale1DBase;
    public readonly Vector2 scale2DBase;
    public readonly Vector3 scale3DBase;

    readonly public Vector3 localVelocityBase;

    public PersistentKSPParticleEmitter(GameObject go, KSPParticleEmitter pe, KSPParticleEmitter templateKspParticleEmitter)
    {
        this.go = go;
        this.pe = pe;

        scale1DBase = templateKspParticleEmitter.shape1D;
        scale2DBase = templateKspParticleEmitter.shape2D;
        scale3DBase = templateKspParticleEmitter.shape3D;

        minEmissionBase = (float)templateKspParticleEmitter.minEmission;
        maxEmissionBase = (float)templateKspParticleEmitter.maxEmission;
        minEnergyBase = templateKspParticleEmitter.minEnergy;
        maxEnergyBase = templateKspParticleEmitter.maxEnergy;

        minSizeBase = (float)templateKspParticleEmitter.minSize;
        maxSizeBase = (float)templateKspParticleEmitter.maxSize;

        localVelocityBase = templateKspParticleEmitter.localVelocity;

        PersistentEmitterManager.Add(this);
    }

    // Detach the emitter from its parent gaemobject and stop its emmission in timer seconds
    public void Detach(float timer)
    {
        this.timer = Time.fixedTime + timer;
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

}
