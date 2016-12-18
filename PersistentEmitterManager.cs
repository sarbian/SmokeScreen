/*
 * Copyright (c) 2017, Sébastien GAGGINI AKA Sarbian, France
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

using System.Collections.Generic;
using System.Linq;
using UnityEngine;

[KSPAddon(KSPAddon.Startup.EveryScene, false)]
internal class PersistentEmitterManager : MonoBehaviour
{
    //public static PersistentEmitterManager Instance { get; private set; }

    public static List<PersistentKSPParticleEmitter> persistentEmitters;
    public static List<PersistentKSPShurikenEmitter> persistentEmittersShuriken;

    private void Awake()
    {
        //PersistentEmitterManager.Instance = this;

        persistentEmitters = new List<PersistentKSPParticleEmitter>();
        persistentEmittersShuriken = new List<PersistentKSPShurikenEmitter>();

        GameEvents.onGameSceneLoadRequested.Add(OnSceneChange);
    }

    private void OnDestroy()
    {
        //Print("OnDestroy");
        GameEvents.onGameSceneLoadRequested.Remove(OnSceneChange);
    }

    public static void Add(PersistentKSPParticleEmitter pkpe)
    {
        persistentEmitters.Add(pkpe);
        EffectBehaviour.AddParticleEmitter(pkpe.pe);

        //print("[SmokeScreen PersistentEmitterManager]: Added 1 PersistentKSPParticleEmitter. Count = " + persistentEmitters.Count);
    }

    public static void Add(PersistentKSPShurikenEmitter pkpe)
    {
        persistentEmittersShuriken.Add(pkpe);
        FloatingOrigin.RegisterParticleSystem(pkpe.pe);
    }

    public static void Remove(PersistentKSPParticleEmitter pkpe)
    {
        EffectBehaviour.RemoveParticleEmitter(pkpe.pe);
        persistentEmitters.Remove(pkpe);

        //print("[SmokeScreen PersistentEmitterManager]: Removed 1 PersistentKSPParticleEmitter. Count = " + persistentEmitters.Count);
    }

    public static void Remove(PersistentKSPShurikenEmitter pkpe)
    {
        FloatingOrigin.UnregisterParticleSystem(pkpe.pe);
        persistentEmittersShuriken.Remove(pkpe);

        //print("[SmokeScreen PersistentEmitterManager]: Removed 1 PersistentKSPParticleEmitter. Count = " + persistentEmitters.Count);
    }

    private void OnSceneChange(GameScenes scene)
    {
        //Print("OnSceneChange");
        for (int i = 0; i < persistentEmitters.Count; i++)
        {
            EffectBehaviour.RemoveParticleEmitter(persistentEmitters[i].pe);
            //Print(" go is " + persistentEmitters[i].go);

            //Destroy(persistentEmitters[i].go);
            if (persistentEmitters[i].go != null && persistentEmitters[i].go.transform.parent != null)
            {
                Destroy(persistentEmitters[i].go);
            }
        }
        persistentEmitters.Clear();

        for (int i = 0; i < persistentEmittersShuriken.Count; i++)
        {
            FloatingOrigin.UnregisterParticleSystem(persistentEmittersShuriken[i].pe);

            if (persistentEmittersShuriken[i].go != null && persistentEmittersShuriken[i].go.transform.parent != null)
            {
                Destroy(persistentEmittersShuriken[i].go);
            }
        }
        persistentEmittersShuriken.Clear();

    }

    public void FixedUpdate()
    {
        //var persistentEmittersCopy = persistentEmitters.ToArray();
        for (int i = 0; i < persistentEmitters.Count; i++)
        {
            PersistentKSPParticleEmitter em = persistentEmitters[i];
            if (em.endTime > 0 && em.endTime < Time.fixedTime)
            {
                em.EmissionStop();
            }

#warning Clearly something is not OK because detached emmiter can generate errors like :
            //          NullReferenceException
            //at(wrapper managed - to - native) UnityEngine.ParticleEmitter:get_particles()
            //at PersistentKSPParticleEmitter.EmitterOnUpdate(Vector3 emitterWorldVelocity)[0x00000] in < filename unknown >:0
            //at ModelMultiParticlePersistFX.FixedUpdate()[0x00000] in < filename unknown >:0


            // A more robust logic need to be put in place for when the PersistentKSPParticleEmitter is detached but the KSPParticleEmitter is destroyed.
            

            // If the gameObject is null clean up the emitter
            if (em.go == null || em.pe == null || em.pe.pe == null)
            {
                //Print("FixedUpdate cleaning null go");
                Remove(em);
                // Make sure
                Destroy(em.go);
                i--;
            }

                // if not and the transform parent is null ( Emitter detached from part so the particle are not removed instantly )
                // then the emitter won't be updated by the effect FixedUpdate Call. So update it here
            else if (em.go.transform.parent == null)
            {
                em.EmitterOnUpdate(Vector3.zero);

                if (em.pe.pe.particleCount == 0)
                {
                    //Print("FixedUpdate cleaning parent go");
                    Remove(em);
                    Destroy(em.go);
                    i--;
                }
            }
        }

        for (int i = 0; i < persistentEmittersShuriken.Count; i++)
        {
            PersistentKSPShurikenEmitter em = persistentEmittersShuriken[i];
            if (em.endTime > 0 && em.endTime < Time.fixedTime)
            {
                em.EmissionStop();
            }
            
            // If the gameObject is null clean up the emitter
            if (em.go == null || em.pe == null)
            {
                Remove(em);
                Destroy(em.go);
                i--;
            }
            // if not and the transform parent is null ( Emitter detached from part so the particle are not removed instantly )
            // then the emitter won't be updated by the effect FixedUpdate Call. So update it here
            else if (em.go.transform.parent == null)
            {
                em.EmitterOnUpdate(Vector3.zero);

                if (em.pe.particleCount == 0)
                {
                    Remove(em);
                    Destroy(em.go);
                    i--;
                }
            }
        }
    }

    private void Print(string s)
    {
        print("[SmokeScreen " + GetType().Name + "] : " + s);
    }
}