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
class PersistentEmitterManager : MonoBehaviour
{

    public static PersistentEmitterManager Instance { get; private set; }

    private static List<PersistentKSPParticleEmitter> persistentEmitters;


    private void Awake()
    {
        PersistentEmitterManager.Instance = this;

        persistentEmitters = new List<PersistentKSPParticleEmitter>();

        GameEvents.onGameSceneLoadRequested.Add(new EventData<GameScenes>.OnEvent(this.OnSceneChange));
    }

    private void OnDestroy()
    {
        GameEvents.onGameSceneLoadRequested.Remove(new EventData<GameScenes>.OnEvent(this.OnSceneChange));
    }

    static public void Add(PersistentKSPParticleEmitter pkpe)
    {        
        persistentEmitters.Add(pkpe);
        EffectBehaviour.AddParticleEmitter(pkpe.pe);
    }

    private void OnSceneChange(GameScenes scene)
    {
        for (int i = 0; i < persistentEmitters.Count; i++)
        {
            EffectBehaviour.RemoveParticleEmitter(persistentEmitters[i].pe);
            if (persistentEmitters[i].go.transform.parent == null)
                Destroy(persistentEmitters[i].go);
        }
        persistentEmitters = new List<PersistentKSPParticleEmitter>();
    }

    void FixedUpdate()
    {
        var persistentEmittersCopy = persistentEmitters.ToArray();
        for (int i = 0; i < persistentEmittersCopy.Length; i++)
        {
            if (persistentEmittersCopy[i].timer > 0 && persistentEmittersCopy[i].timer < Time.fixedTime)
            {
                persistentEmittersCopy[i].EmissionStop();
            }

            if (persistentEmittersCopy[i].go == null  || (persistentEmittersCopy[i].go.transform.parent == null && persistentEmittersCopy[i].pe.pe.particles.Count() == 0))
            {
                EffectBehaviour.RemoveParticleEmitter(persistentEmittersCopy[i].pe);
                persistentEmitters.Remove(persistentEmittersCopy[i]);
                if (persistentEmittersCopy[i].go != null)
                    Destroy(persistentEmittersCopy[i].go);
            }
        }
    }

    private void Print(string s)
    {
        MonoBehaviour.print(this.GetType().Name + " : " + s);
    }


}


