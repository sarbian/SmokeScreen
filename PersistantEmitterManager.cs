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
class PersistantEmitterManager : MonoBehaviour
{

    public static PersistantEmitterManager Instance { get; private set; }

    private static List<PersistantKSPParticleEmitter> persistantEmitters;


    private void Awake()
    {
        PersistantEmitterManager.Instance = this;

        persistantEmitters = new List<PersistantKSPParticleEmitter>();

        GameEvents.onGameSceneLoadRequested.Add(new EventData<GameScenes>.OnEvent(this.OnSceneChange));
    }

    private void OnDestroy()
    {
        GameEvents.onGameSceneLoadRequested.Remove(new EventData<GameScenes>.OnEvent(this.OnSceneChange));
    }

    static public void Add(PersistantKSPParticleEmitter pkpe)
    {        
        persistantEmitters.Add(pkpe);
        EffectBehaviour.AddParticleEmitter(pkpe.pe);
    }

    private void OnSceneChange(GameScenes scene)
    {
        for (int i = 0; i < persistantEmitters.Count; i++)
        {
            EffectBehaviour.RemoveParticleEmitter(persistantEmitters[i].pe);
            if (persistantEmitters[i].go.transform.parent == null)
                Destroy(persistantEmitters[i].go);
        }
        persistantEmitters = new List<PersistantKSPParticleEmitter>();
    }


    void FixedUpdate()
    {
        List<PersistantKSPParticleEmitter> persistantEmittersCopy = new List<PersistantKSPParticleEmitter>(persistantEmitters);
        for (int i = 0; i < persistantEmittersCopy.Count; i++)
        {
            if (persistantEmittersCopy[i].go.transform.parent == null && persistantEmittersCopy[i].pe.pe.particles.Count() == 0)
            {
                EffectBehaviour.RemoveParticleEmitter(persistantEmittersCopy[i].pe);
                persistantEmitters.Remove(persistantEmittersCopy[i]);
                Destroy(persistantEmittersCopy[i].go);
            }
        }

    }

    private void print(String s)
    {
        MonoBehaviour.print(this.GetType().Name + " : " + s);
    }


}


