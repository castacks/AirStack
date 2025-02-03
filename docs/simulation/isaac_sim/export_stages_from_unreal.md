# Export Unreal Engine to Isaac Sim

A robot needs a scene to interact in. A scene can be created in any 3D modeling program, though we have found it easiest to export stages from Unreal Engine. This document explains how to export an Unreal Engine environment to an Isaac Sim stage, then how to convert the stage to a physics-enabled scene. 


## Exporting Unreal Engine Environments to Isaac Sim Stages
Generally, Unreal Engine environments can be found on Epic Games' [Fab Marketplace](https://www.fab.com/). For example, the [Open World Demo Collection](https://www.fab.com/listings/3262ab8f-f64a-4124-8efd-82cb19df6249) is a free collection of outdoor environments.


The below video explains how to export an Unreal Engine environment to an Isaac Sim stage.

<iframe width="840" height="473" src="https://www.youtube.com/embed/y9qP7wcinHM?si=i5TVd1CFbJUYhWz7" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

You can save this file as `[YOUR_ENVIRONMENT_NAME].stage.usd`.


### Export Tips

**Complexity:**
Generally, it works best to export levels that are designed for UE4.27 and below. The advanced rendering features from UE5, i.e. nanite and lumen, aren't compatible with Omniverse. 
If the level is optimized for the older UE4, then it's more compatible.

Omniverse doesn't perform well with large amounts of vegetation. Anything with complex vegetation takes a long long time to load. Procedural foliage doesn't export well either. Simple geometries like buildings and rocks work better. 

That said you can still achieve photorealism by substituting complex geometries for high quality textures. Isaac seems to do fine with high quality textures.

**Optimization:** After exporting, edit the file with [USD Composer](https://docs.omniverse.nvidia.com/composer/latest/index.html) and run the [Scene Optimizer extension](https://docs.omniverse.nvidia.com/extensions/latest/ext_scene-optimizer.html) for faster performance. USD Composer can be installed via [Omniverse Launcher](https://docs.omniverse.nvidia.com/launcher/latest/index.html).

**Verify the Scale:**  The Omniverse exporter exports in centimeters, but Isaac Sim natively works in meters. For consistency, follow these steps to [change the scene units to be meters](https://forums.developer.nvidia.com/t/how-to-change-units-of-the-grid-from-centimeters-to-meters/301285#:~:text=Find%20the%20%E2%80%9CMeters%20Per%20Unit%E2%80%9D%20property%20and%20set%20it%20to%201%20for%20meters).

To check the scale of the scene, you can add a cube in Isaac Sim and compare it to the exported scene. The cube is 1m x 1m x 1m.

## Turn a Stage into a Physics-Enabled Scene

Adding physics to the stage is as simple as adding a `Physics` property with the "Colliders Preset", as described in the [Isaac docs](https://docs.omniverse.nvidia.com/isaacsim/latest/gui_tutorials/tutorial_intro_simple_objects.html#adding-physics-properties).
Then save the scene as `[YOUR_ENVIRONMENT_NAME].scene.usd` to clarify that it's a physics-enabled scene.

You're now ready to add robots to the scene on the next page.