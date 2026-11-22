Open **http://127.0.0.1:18767/**. Its two buttons select the existing factory twin or the separate board variant. Physical finger changes never select an environment.

From the repository root:

```bash
digital_twin/sim.sh board --fingers wide
digital_twin/sim.sh board --fingers original
digital_twin/sim.sh factory --fingers original
```

Environment and fingers are independent. Use `--rebuild` after changing `profiles.json`; otherwise a matching running variant is reused. The launcher replaces only its own processes. Initialization is bounded and failures name the VM log; re-run to recover. The original factory on port 18766 and its active session remain intact.

The empty RL plate retains its photo-derived registration: 32×32 studs, outer row taped on every side, 30×30 exposed. No photographed plate bricks or RL example stack are added. Plate height, thickness and contact physics remain unmeasured.

The passive guide spacing is **assembled-layer measured width + total guide clearance** in `pocket_fit`. Enter the maximum outside width across all four rows in `measured_assembled_span_m` with `measurement_evidence`, then rebuild. The operator approved a **95.8 mm clear pocket span**. Its nominal basis is 95.4 mm brick sum + 0.4 mm total clearance; that approval is not recorded as a measured assembled-layer width. The 0.2 mm per side is a commissioning design allowance, not a verified manufacturing tolerance. The two end walls move inward; the ceiling, back wall, 20 mm inward reach, 3.2 mm wall thickness, original tapered profiles, mounting spine and open underside are retained. Brick meshes and taught poses are not scaled or moved to force a fit.

The obsolete stock-pad portion of each terminal collision box is removed at the existing metal backplane; the metal supports remain.

This side-spacing revision uses `ceiling.mode = preserve_current`: the running 3.2 mm ceiling remains unchanged. The pending fixed-top **17.45 mm + verified fingertip-to-brick-bottom clearance** calculation is retained for a later explicit selection. It requires a verified clearance before replacing a working variant. The nominal stud height remains 1.7 mm; estimated display-table heights are not physical measurements. Receiving-layer vertical/stud clearance and loaded release are not verified by a passive-span change.

The nominal 95.4×63.2 mm whole-layer center is the existing middle-column pose `{n}_1`. Native open/close targets remain 70/60 mm; CAD mounting offsets make the inside gap different. Simulated joint limits preserve 1 mm clearance between opposing caps. Incompatible measured hand widths retain an explicitly labeled simulated opening; selection never changes automatically. Real controller settings, inventory and taught poses are untouched.

`environment.json` records dimensions and fit checks; `digital_twin.usdc` stores the variant snapshot. With VM Isaac Python, `audit.py OUTPUT_DIRECTORY` independently checks the saved USD dimensions, mesh topology and collision-free simulated stroke. Geometric fit does not validate loaded lifting, friction, mass or plate contact.

The variant follows the existing twin recording and shares camera display files read-only. It opens no SDK connection. Original timestamps/stale status are retained. With unavailable hand telemetry, the preview starts at an explicitly **simulated** 70 mm native width. SDK readers stay paused for Desk.
