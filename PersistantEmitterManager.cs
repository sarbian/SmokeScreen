/*
 * Author: Sébastien GAGGINI AKA Sarbian, France
 * License: BY: Attribution 4.0 International (CC BY 4.0): http://creativecommons.org/licenses/by/4.0/
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

    private static List<KSPParticleEmitter> emitters;
    private static List<GameObject> emittersGameObjects;

    private void Awake()
    {
        PersistantEmitterManager.Instance = this;
        emitters = new List<KSPParticleEmitter>();
        emittersGameObjects = new List<GameObject>();
        GameEvents.onGameSceneLoadRequested.Add(new EventData<GameScenes>.OnEvent(this.OnSceneChange));
    }

    private void OnDestroy()
    {
        GameEvents.onGameSceneLoadRequested.Remove(new EventData<GameScenes>.OnEvent(this.OnSceneChange));
    }

    static public void Add(KSPParticleEmitter kpe, GameObject go)
    {
        emitters.Add(kpe);
        emittersGameObjects.Add(go);
        EffectBehaviour.AddParticleEmitter(kpe);
    }

    private void OnSceneChange(GameScenes scene)
    {
        for (int i = 0; i < emitters.Count; i++)
        {
            EffectBehaviour.RemoveParticleEmitter(emitters[i]);
            if (emittersGameObjects[i].transform.parent == null)
                Destroy(emittersGameObjects[i]);
        }
        emitters = new List<KSPParticleEmitter>();
        emittersGameObjects = new List<GameObject>();
    }

        
    void FixedUpdate()
    {
        List<KSPParticleEmitter> emittersCopy = new List<KSPParticleEmitter>(emitters);
        List<GameObject> emittersGameObjectsCopy = new List<GameObject>(emittersGameObjects);
        for (int i = 0; i < emittersCopy.Count; i++)
        {
            if (emittersGameObjectsCopy[i].transform.parent == null && emittersCopy[i].pe.particles.Count() == 0)
            {
                EffectBehaviour.RemoveParticleEmitter(emittersCopy[i]);
                emitters.Remove(emittersCopy[i]);
                emittersGameObjects.Remove(emittersGameObjectsCopy[i]);
                Destroy(emittersGameObjectsCopy[i]);
            }
        }

    }

    private void print(String s)
    {
        MonoBehaviour.print(this.GetType().Name + " : " + s);
    }


}

