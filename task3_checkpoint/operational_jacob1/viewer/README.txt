BRICKFLOW — UPDATED OPERATIONAL VIEWER

Serve the simulation root (one directory above this viewer), then open viewer/.
The root provides geometry-manifest.json, engine/operational-engine.js and run.json.
Startup computes a new deterministic run in a browser Worker. Regenerate run
repeats the same audited default scenario; run.json is an optional fallback.
No existing replay is loaded by this viewer.

The static shell is derived from the accepted v4 layout. All illustrative brick
loads, warehouse pallets, staged packs, truck cargo and reference forklifts have
been removed. Product bricks and pallets are rendered only from sampled engine
records. Frozen storage slot centers, aisle geometry and fixed gantry structure
retain their accepted coordinates. New N/S kiln-cart rails and upstream bridge
use the appended upstream_kiln_cart manifest section.

Open process covers, overhead structures and truck roofs are cut away by default.
Functional upstream reject-channel retaining walls remain visible. Well walls
are transparent for observing descent. Optical cutaways do not change support
or collision geometry in the model.

CONTROLS
Play/pause; restart at zero; 0.1x/0.25x/0.5x/1x/5x/20x/100x playback.
The timeline seeks using absolute simulation seconds and recomputes sampled state.
Choose any of the 18 stations. Formation, layer-buffer, pallet-stacking and pickup
camera presets use the selected station. Additional views cover kiln cart/claws,
routing, shared storage, rejects, dispatch and top-down facility.
Event type and event selection provide direct jumps; next/previous and dedicated
layer/pallet/gantry/skip buttons pause before a matching event for inspection.

DISPLAY DATUMS
Brick poses are centers of 245 x 69 x 67 mm material meshes.
Pallet pose is the wood body reference: runners bottom = Y - 0.060 m,
usable top deck = Y + 0.060 m, agreeing with the model's contact datum.
Well deck pose Y is its supporting top surface.
Layer-head pose is the center of its 0.12 m tool body; its underside aligns
with the carried layer's brick top. An open perimeter frame leaves brick IDs
visible. Gantry tool poses use the engine's pallet-support frame reference.
These are kinematic inspection silhouettes, not qualified supplier mechanisms.

The skip head uses one rigid mesh, lifted and rotated using engine-sampled
headLift/headYaw. Its additional static review poses are hidden. Removed reject
IDs stay in the material ledger; the physical body uses an illustrative counted
bulk-fill mesh, not a claim to simulate breakage or granular packing.

VERIFICATION
node verify_shell.cjs executes actual static constructors with a no-op renderer;
it verifies 128 slot markers, 12 inspection cameras, zero static product meshes
and default open/cutaway views. This lightweight check uses no GPU.
Root owns browser review and independent process validation.
