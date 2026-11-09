SmokeScreen
===========

A plugin for Kerbal Space Program that adds an improved version of the MODEL_MULTI_PARTICLE effect.
Many thanks to Nothke for the ideas and test, and to Egg for his commits and helpful code review.

SmokeScreen adds the `MODEL_MULTI_SHURIKEN_PERSIST` effect. It takes a particle model (a `.mu` with a
`KSPParticleEmitter`, the same models stock `MODEL_MULTI_PARTICLE` uses) and does the emission itself, so that:

- the particles of a part that is destroyed or decoupled stay in the scene until they die;
- every emission value can follow curves of several inputs, not only the effect's power;
- the particles can grow, fade, feel drag, gravity and buoyancy, and collide.

It requires KSP 1.12. The Toolbar mod is optional: it adds a button for the debug window (also opened with
Mod + P in flight).

Building
--------

Open `SmokeScreen.csproj` or run `dotnet build SmokeScreen.csproj -c Release`. The references are taken from
`$(KSPDIR)\KSP_x64_Data\Managed`; set the `KSPDIR` environment variable (or pass `-p:KSPDIR=<path>`) to a KSP
install. When `KSPDIR` is set, the build also copies `SmokeScreen.dll` and its `.pdb` to `GameData\SmokeScreen`
in that install.

GitHub Actions builds every push to `master` and every pull request against the KSP 1.12.5 assemblies of
[KSPModdingLibs](https://github.com/KSPModdingLibs/KSPLibs), and a pull request gets a comment with the zip of its
`GameData/SmokeScreen` folder. The `create-release` workflow, run by hand with a `major.minor.patch.build` version,
builds that version, commits it to `Properties/AssemblyInfo.cs`, tags `v<version>` and publishes a GitHub release
with the zip and `SmokeScreen.version`, which the build generates from `SmokeScreen.version.in`.

The effect
----------

```
EFFECTS
{
    running
    {
        MODEL_MULTI_SHURIKEN_PERSIST
        {
            name = plume
            modelName = Squad/FX/Monoprop_small
            transformName = thrustTransform
            emission
            {
                power = 0.0 0.0
                power = 1.0 1.0
                density = 0.0 0.5
                density = 1.0 1.0
            }
            speed = 0.0 0.8
            speed = 1.0 1.0
        }
    }
}
```

One emitter is created for each transform named `transformName` in the part's model (`partTransform` means
the part itself). Each emitter copies the model's emitter values (`minEmission`, `maxEmission`, `minEnergy`,
`maxEnergy`, `minSize`, `maxSize`, the shape, `localVelocity`, `worldVelocity`, `force`, `rndForce`,
`rndVelocity`, `angularVelocity`, `rndAngularVelocity`, `sizeGrow`, `color`, `colorAnimation`) and the curves
below scale them.

### Curves

A curve is a node named after the value, with one key list per input:

| Input | Value |
|---|---|
| `power` | the power the part sends with the effect event, 0 to 1 |
| `density` | the atmospheric density at the part in kg/m³, raised to the global `atmDensityExp` |
| `mach` | the part's Mach number |
| `parttemp` | the part's temperature in K |
| `externaltemp` | the vessel's external temperature in K |
| `time` | the time in s, modulo `timeModulo`; during a single-emit timer, the time left |

Each key is `input = <input value> <output value>`, as in FXCurve: keys have flat tangents, so the value eases
in and out between keys. A key list named `log<input>` (`logdensity`, `logmach`...) is evaluated on the
base-10 logarithm of the input. For backward compatibility, a value `<curve> = <input> <output>` directly in
the effect node is the `power` key list.

There are two kinds of curves. In a **multiplicative** curve the results of the inputs are multiplied, and an
input without keys counts as 1. In an **additive** curve they are added, and an input without keys counts
as 0.

| Curve | Kind | What it scales |
|---|---|---|
| `emission` | × | the emission rate (`minEmission`, `maxEmission`), times `emissionMult` |
| `energy` | × | the lifetime (`minEnergy`, `maxEnergy`) |
| `speed` | × | `localVelocity` and `worldVelocity` (times the scale) |
| `size` | × | the start size (`minSize`, `maxSize`, times the scale) |
| `scale` | × | the emitter shape (times the scale) |
| `force` | + | multiplies the model's `force`; the force over lifetime is that force ± `rndForce` / 2 |
| `grow` | + | growth rate per second of each particle: its size is multiplied by (1 + grow) each second |
| `linGrow` | + | linear growth: the size grows by `linGrow` × the mean start size per second |
| `logGrow` | + | logarithmic growth: after t seconds the size is about (log(logGrow × t + 1) × `logGrowScale` + 1) × the start size |
| `logGrowScale` | × | see `logGrow` |
| `sizeClamp` | + | added to the `sizeClamp` value below |
| `offset` | + | moves the emitter along `offsetDirection` (times the scale) |
| `alpha` | × | the alpha of the particles over their life (1 = unchanged) |
| `linAlphaDecay` | + | linear fade of the alpha over the particle's life |
| `logAlphaDecay` | + | logarithmic fade of the alpha over the particle's life |
| `saturationMult` | × | the colour's saturation |
| `brightnessMult` | × | the colour's brightness |
| `alphaMult` | × | the colour's alpha |
| `initalVelocityOffsetMaxRadius` | + | added to `randomInitalVelocityOffsetMaxRadius` |
| `randConeEmit` | + | spreads the new particles' velocity in a cone around the emitter's axis (larger is wider, up to 90°) |
| `vPosOffset` | + | moves new particles along their velocity by this distance |
| `vRandPosOffset` | + | moves new particles along their velocity by a random distance up to this value |
| `xyForce` | × | damps the particles' velocity across the emitter's axis (1 = no damping, applied 50 times a second) |
| `zForce` | × | the same along the emitter's axis |

`randConeEmit`, `vPosOffset`, `vRandPosOffset`, `xyForce` and `zForce` act only on models that simulate in local
space.

When `alpha`, `linAlphaDecay` and `logAlphaDecay` are all at their defaults, the colour over lifetime is not
changed. Otherwise the particles fade over their life with alpha × (1 − linAlphaDecay × age − log(logAlphaDecay
× age + 1)), the age going from 0 to 1, and their colour darkens with the alpha.

`angle` and `distance` are plain FXCurves (`angle = <degrees> <value>`, `distance = <m> <value>`) of the
angle between the emitter and the camera and of the camera's distance. Their product scales the renderer's
maximum particle size.

### Values

| Key | Default | Meaning |
|---|---|---|
| `modelName` | | the particle model |
| `transformName` | | the transforms to emit from, one emitter each |
| `localPosition` | 0, 0, 0 | the emitter's position in the transform's local space, **in model units**: it scales with the part's model (rescaleFactor and the model's scale), unlike `offset` |
| `localRotation` | 0, 0, 0 | the emitter's rotation in the transform's local space, Euler angles in degrees |
| `offsetDirection` | 0, 0, 1 | the direction the `offset` curve moves the emitter |
| `fixedScale` | 1 | scales size, speed, shape and offset. ProceduralSRB's bell scale and TweakScale's scale are applied on top |
| `emissionMult` | 1 | multiplies the emission rate |
| `sizeClamp` | 50 | the largest size in m a particle reaches by growing, including the model's own `sizeGrow`. A `sizeClamp = N` line is also read as the `power` key list of the `sizeClamp` curve (a single value is a constant), so it counts twice: the clamp is 2N. Configs written since 2015 rely on that |
| `layer` | TransparentFX | the layer of the emitter's objects. An unknown name falls back to TransparentFX |
| `renderMode` | Billboard | read and saved, but not applied: the particles keep the model's render mode |
| `particleCountLimit` | 1000 | the most particles one emitter keeps |
| `timeModulo` | 10 | the period of the `time` input in s |
| `singleEmitTimer` | 0 | after a single event (`part.Effect(name)` with no power), the effect runs at power 1 for this many seconds. With 0, the single event emits one second of particles at once |
| `emitOnUpdate` | false | emit in LateUpdate, in step with the frames, instead of FixedUpdate. Forced on in on-rails warp |
| `decluster` | false | spread the particles emitted in one update along their path, so that they don't clump at low frame rates |
| `randomInitalVelocityOffsetMaxRadius` | 0 | adds to new particles a random velocity in m/s in a disc across their velocity |
| `physical` | false | apply gravity, buoyancy and drag to the particles |
| `initialDensity` | 0.6 | the particles' initial density in kg/m³, for `physical` |
| `dragCoefficient` | 0.1 | the particles' drag coefficient, for `physical` |
| `collide` | false | make the particles collide with parts and the ground |
| `collideRatio` | 0 | the share of the normal velocity kept after a collision |
| `stickiness` | 0.9 | how much of the tangential spread is lost in a collision |

`physical` and `collide` cost a ray cast or a density computation per particle (spread over four updates); the
debug window can turn them off for every effect at once.

Global settings
---------------

`GameData/SmokeScreen/SmokeScreen.cfg` holds the defaults; the debug window saves your changes to
`GameData/SmokeScreen/PluginData/SmokeScreen.cfg`, which is read instead when it exists.

| Key | Default | Meaning |
|---|---|---|
| `maximumActiveParticles` | 1000 | above this many particles in the scene, the emitters with more than 30 particles drop some (8000 when the file doesn't set it) |
| `atmDensityExp` | 1 | the exponent applied to the density input |
| `globalCollideDisable` | False | turn `collide` off on every effect |
| `globalPhysicalDisable` | False | turn `physical` off on every effect |
| `forceEmitOnUpdate` | False | turn `emitOnUpdate` on for every effect |
| `forceDecluster` | False | turn `decluster` on for every effect |
| `shareManualInput` | False | the debug window's manual input values apply to every effect |

Launch effects
--------------

`KM_PreLaunchEffect_SmkS` and `KM_PostLaunchEffect_SmkS` (from dtobi's Cool Rockets) play the
`KSPParticleEmitter`s named `effectName` of a model before and after launch. The flight menu's "Toggle
<effectName>" button and the "Toggle" action turn the effect on and off.
