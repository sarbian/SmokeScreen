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
    public float baseMaxSize;

    public float timer = 0;

    public PersistentKSPParticleEmitter(GameObject go, KSPParticleEmitter pe, float ms)
    {
        this.go = go;
        this.pe = pe;
        baseMaxSize = ms;
        PersistentEmitterManager.Add(this);
    }

    public PersistentKSPParticleEmitter(GameObject go, KSPParticleEmitter pe)
        : this(go, pe, 0)
    {
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
