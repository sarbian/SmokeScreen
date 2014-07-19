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

[KSPAddon(KSPAddon.Startup.EveryScene, false)]
internal class PersistentEmitterManager : MonoBehaviour
{
    //public static PersistentEmitterManager Instance { get; private set; }

    public static List<PersistentKSPParticleEmitter> persistentEmitters;

    private void Awake()
    {
        //PersistentEmitterManager.Instance = this;

        persistentEmitters = new List<PersistentKSPParticleEmitter>();

        GameEvents.onGameSceneLoadRequested.Add(new EventData<GameScenes>.OnEvent(this.OnSceneChange));
    }

    private void OnDestroy()
    {
        //Print("OnDestroy");
        GameEvents.onGameSceneLoadRequested.Remove(new EventData<GameScenes>.OnEvent(this.OnSceneChange));
    }

    public static void Add(PersistentKSPParticleEmitter pkpe)
    {
        persistentEmitters.Add(pkpe);
        EffectBehaviour.AddParticleEmitter(pkpe.pe);
        //print("[SmokeScreen PersistentEmitterManager]: Added 1 PersistentKSPParticleEmitter. Count = " + persistentEmitters.Count);
    }

    public static void Remove(PersistentKSPParticleEmitter pkpe)
    {
        EffectBehaviour.RemoveParticleEmitter(pkpe.pe);
        persistentEmitters.Remove(pkpe);
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
            if (persistentEmitters[i].go != null)
            {
                Destroy(persistentEmitters[i].go);
            }
        }
        persistentEmitters = new List<PersistentKSPParticleEmitter>();
    }

    public void FixedUpdate()
    {
        var persistentEmittersCopy = persistentEmitters.ToArray();
        for (int i = 0; i < persistentEmittersCopy.Length; i++)
        {
            if (persistentEmittersCopy[i].endTime > 0 && persistentEmittersCopy[i].endTime < Time.fixedTime)
            {
                persistentEmittersCopy[i].EmissionStop();
            }
                        
            // If the gameObject is null clean up the emitter 
            if (persistentEmittersCopy[i].go == null)
            {
                //Print("FixedUpdate cleanning null go");
                Remove(persistentEmittersCopy[i]);
                // Make sure
                Destroy(persistentEmittersCopy[i].go); 
            }
            // if not and the tranform parent is null ( Emitter detached from part so the particle are not removed instantly )
            // then the emitter won't be updated by the effect FixedUpdate Call. So update it here
            else if (persistentEmittersCopy[i].go.transform.parent == null)
            {
                persistentEmittersCopy[i].EmitterOnUpdate(Vector3.zero);

                if (persistentEmittersCopy[i].pe.pe.particles.Count() == 0)
                {
                    //Print("FixedUpdate cleanning parent go");
                    Remove(persistentEmittersCopy[i]);
                    Destroy(persistentEmittersCopy[i].go);
                }
            }
        }
    }

    private void Print(string s)
    {
        MonoBehaviour.print("[SmokeScreen " + this.GetType().Name + "] : " + s);
    }
}