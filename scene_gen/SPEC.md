# Scene generator design
## Settings
high-level settings are stored in high-level configs, which then compile to low-level settings, which are fed to the generator.

Locale:
- Urban
- Suburban
- Rural

Types of disasters:
- Earthquake
- Tornado
- Hurricane
- Fire
- Flood
Severity: Continuous value from 0-1

"severity" is a generic knob, and the compilation needs to specify how it modulates the low-level parameters.
This will be different for each type of disaster, and can also differ between locales

## Asset sources
Nucleus, Objaverse, Local
Assets:
- usd, fbx, gltf
- meshes, textures, materials
- May be:
	- standalone props
	- modular components that must be constructed first
	- generic textures/materials that are applied during damage
		- interior materials
		- scorch or damage materials
master index of all usable assets
Asset pack yaml files take subsets from this index, organize them, and allow users to assign metadata
 
## Workflow/Usage
### Final usage
Entrypoint 1: Offline scene generation
- Given a high-level config, generate the scene and bake it into one usd (keep prims separate).
	- Save the layout, pristine scene, and final disaster scene in a gitignored cache
	- Keep these files in a 2-tier tree-style data structure keyed by features in the high-level config
		- seed, asset pack, locale -> layout and pristine scene
		- add the disaster severity -> final disaster scene (parent is the pristine version)
- users may upload to nucleus

Entrypoint 2: airstack up
- Simply use a link to a stage
- Can be a local path or nucleus path
- What happens here:
	- Dynamic placement of targets (victims)
- Actual deployment of search methods and such

### Development
Need to be able to iterate fast on each aspect of the pipeline. Thus need tools to preview assets, constructed buildings, damage effects on buildings, layout, etc.

## Pipeline
Stage A — Bake the archetype library  (once; exhaustive; layout-independent)
	Steps:
		1. Compile config (disaster type + parameters)
		2. Measure assets / use cached
		3. For every displaceable asset TYPE × every severity LEVEL:
			a. Build one clean instance of that type
			b. Apply the disaster's damage model to it
			c. Simulate its physics
		4. Export each result individually as a self-contained USD with the
		   disaster baked in  ->  <disaster>/<type>_<level>.usd
	Invariants:
		- EXHAUSTIVE: all type × level combinations are baked regardless of
		  whether a given scene uses them; the library is built once and
		  reused across every scene, seed, and run.
		- Layout-independent: assets are built in isolation, not from a layout.
		- Only assets the disaster DISPLACES (e.g. buildings, trees) are baked;
		  surface-only damage is deferred to Stage B, step 6.
		- All assets are USD.

Stage B — Assemble a scene
	Steps:
		1. Compile config (disaster type + parameters)
		2. Create layout + build the disaster model
		3. Start Isaac Sim
		4. Place roads, ground, and other unaffected/live geometry
		5. For each damaged building / tree / displaceable asset:
		   reference the matching pre-baked archetype as a STATIC asset,
		   chosen by (type, severity level the field gives its location)
		6. Apply damage-effect materials to the surfaces the disaster reached
		   but that were NOT baked — roads, props, and the ground material —
		   driven by the same severity field
	Invariants:
		- No per-scene physics: all displacement was baked in Stage A.
		- Repeated archetypes are instanced so identical references share
		  geometry.
		- Convert any damaged asset to USD before placing.

Stage C — airstack up
	Steps:
		1. Load pre-baked scene
		2. Place targets (usually human victims), noting their ground truth locations
		3. Maybe some dynamic setting of atmospheric/lighting conditions
		4. Run downstream methods

## Best Practice
The guiding principle should be simplicity. When convenient, use a simpler, general mechanism with parameters rather than introducing bespoke new logic for a specific asset in a specific locale for a specific disaster, etc.
However, this is purely for the sake of simpler code. DO NOT sacrifice quality of scenes to adhere to this practice.