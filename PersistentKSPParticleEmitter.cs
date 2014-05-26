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

using UnityEngine;

// TODO : handle the relation with PersistentEmitterManager inside the class
public class PersistentKSPParticleEmitter
{
    public GameObject go;

    public KSPParticleEmitter pe;

    public bool fixedEmit = false;

    public float timer = 0;

    public double fraction = 0;

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

        minEmissionBase = (float)templateKspParticleEmitter.minEmission;
        maxEmissionBase = (float)templateKspParticleEmitter.maxEmission;
        minEnergyBase = templateKspParticleEmitter.minEnergy;
        maxEnergyBase = templateKspParticleEmitter.maxEnergy;

        minSizeBase = (float)templateKspParticleEmitter.minSize;
        maxSizeBase = (float)templateKspParticleEmitter.maxSize;

        localVelocityBase = templateKspParticleEmitter.localVelocity;
        worldVelocityBase = templateKspParticleEmitter.worldVelocity;

        forceBase = templateKspParticleEmitter.force;

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

    private void Print(string s)
    {
        MonoBehaviour.print(this.GetType().Name + " : " + s);
    }

}
