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

    public PersistentKSPParticleEmitter(GameObject go, KSPParticleEmitter pe, float ms)
    {
        this.go = go;
        this.pe = pe;
        baseMaxSize = ms;
    }

    public PersistentKSPParticleEmitter(GameObject go, KSPParticleEmitter pe)
    {
        this.go = go;
        this.pe = pe;
        this.baseMaxSize = 1;
    }

}
