"""Lattice/support and registration checks, not physical clutch validation."""
import math
import unittest

try:
    from .stack_layout import LayoutError, compile_world_poses, generate_layout
except ImportError:
    from stack_layout import LayoutError, compile_world_poses, generate_layout


def config(layers=3):
    return {"baseplate": {"grid_shape": [32, 32], "stud_pitch_m": .008,
                          "grid_shape_status": "unverified", "stud_pitch_status": "nominal",
                          "T_O_plate": None},
            "stack": {"center_mode": "plate_center", "layer_count": layers,
                      "layer_pitch_m": .0096, "orientation_quarter_turns": 0,
                      "placement_order": "layer_major"}}


class StackLayoutTests(unittest.TestCase):
    def test_alternating_support_patterns_and_cantilever_risk_are_explicit(self):
        layout = generate_layout(config())
        self.assertEqual(len(layout["bricks"]), 36)
        self.assertEqual(layout["stack_center_stud"], [15.5, 15.5])
        indexed = {brick["id"]: brick for brick in layout["bricks"]}
        for k in range(3):
            for n in range(4):
                for m in range(3):
                    brick = indexed[f"b_{n}_{m}_{k}"]
                    support = brick["support"]
                    self.assertEqual(len(brick["studs"]), 8)
                    if k == 0:
                        self.assertEqual(support["potential_match_count"], 8)
                        self.assertEqual(support["contributors"], ["baseplate"])
                    elif n in (0, 3):
                        self.assertEqual(support["potential_match_count"], 2)
                        self.assertEqual(len(support["contributors"]), 2)
                        self.assertEqual(support["support_affine_rank"], 1)
                        self.assertFalse(support["center_in_support_hull"])
                        self.assertTrue(support["requires_cantilever_clutch_validation"])
                    else:
                        self.assertEqual(support["potential_match_count"], 4)
                        self.assertEqual(len(support["contributors"]), 4)
                        self.assertEqual(support["support_affine_rank"], 2)
                        self.assertTrue(support["center_in_support_hull"])
                    self.assertFalse(support["mechanical_stability_validated"])
                    self.assertTrue(all(not match["engagement_verified"] for match in support["matches"]))
        self.assertEqual(len(layout["summary"]["cantilever_validation_bricks"]), 12)

    def test_rotates_around_shared_center_and_keeps_nominal_grid_spacing(self):
        layout = generate_layout(config())
        center = layout["stack_center_stud"]
        indexed = {brick["id"]: brick for brick in layout["bricks"]}
        a, b, c = (indexed[f"b_0_0_{k}"] for k in range(3))
        relative = [a["center_stud"][i]-center[i] for i in range(2)]
        self.assertEqual([b["center_stud"][i]-center[i] for i in range(2)], [-relative[1], relative[0]])
        self.assertEqual([c["center_stud"][i]-center[i] for i in range(2)], [-relative[0], -relative[1]])
        self.assertAlmostEqual(b["yaw_rad"], math.pi/2)
        self.assertEqual(layout["nominal_layer_footprint_studs"], [12, 14])
        self.assertAlmostEqual(indexed["b_0_1_0"]["bottom_center_P_m"][0]-a["bottom_center_P_m"][0], .032)
        self.assertNotAlmostEqual(indexed["b_0_1_0"]["bottom_center_P_m"][0]-a["bottom_center_P_m"][0], .0318)

    def test_parity_edges_and_missing_center_fail(self):
        bad = config()
        bad["stack"].update(center_mode="explicit_stud", center_stud=[15, 15])
        with self.assertRaisesRegex(LayoutError, "half-integer"):
            generate_layout(bad)
        bad["stack"]["center_stud"] = None
        with self.assertRaisesRegex(LayoutError, "resolved"):
            generate_layout(bad)
        bad["stack"]["center_stud"] = [.5, .5]
        with self.assertRaisesRegex(LayoutError, "outside"):
            generate_layout(bad)

    def test_custom_order_checks_exact_inventory_and_support_dependencies(self):
        cfg = config(2)
        layout = generate_layout(cfg)
        order = layout["placement_order"]
        cfg["stack"]["placement_order"] = list(reversed(order))
        with self.assertRaisesRegex(LayoutError, "supporting bricks"):
            generate_layout(cfg)
        cfg["stack"]["placement_order"] = order[:-1]+[order[0]]
        with self.assertRaisesRegex(LayoutError, "exactly once"):
            generate_layout(cfg)
        cfg["stack"]["placement_order"] = list(reversed(order[:12]))+order[12:]
        self.assertEqual(generate_layout(cfg)["placement_order"], cfg["stack"]["placement_order"])

    def test_world_registration_unknown_and_nominal_flags_do_not_make_commands(self):
        cfg = config(1)
        with self.assertRaisesRegex(LayoutError, "unresolved"):
            compile_world_poses(cfg)
        cfg["baseplate"]["T_O_plate"] = [[1,0,0,.4],[0,1,0,-.1],[0,0,1,.05],[0,0,0,1]]
        with self.assertRaisesRegex(LayoutError, "unverified"):
            compile_world_poses(cfg)
        preview = compile_world_poses(cfg, require_verified=False)
        self.assertTrue(preview["preview_only"])
        self.assertFalse(preview["registered_geometry_verified"])
        self.assertIsNone(preview["tool_poses"])
        self.assertFalse(preview["training_ready"])
        cfg["baseplate"].update(registration_status="verified", grid_shape_status="measured", stud_pitch_status="measured")
        registered = compile_world_poses(cfg)
        local = generate_layout(cfg)["bricks"][0]["bottom_center_P_m"]
        self.assertEqual(registered["poses"][0]["brick_bottom_center_O_m"], [local[0]+.4, local[1]-.1, .05])
        self.assertIsNone(registered["tool_poses"])
        self.assertFalse(registered["physical_engagement_validated"])

    def test_stale_layout_and_improper_transform_rejected(self):
        cfg = config(1)
        stale = generate_layout(cfg)
        cfg["baseplate"]["T_O_plate"] = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        cfg["baseplate"]["stud_pitch_m"] = .00801
        with self.assertRaisesRegex(LayoutError, "stale"):
            compile_world_poses(cfg, stale, require_verified=False)
        for transform in ([[2,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
                          [[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]):
            cfg["baseplate"]["T_O_plate"] = transform
            with self.assertRaises(LayoutError):
                compile_world_poses(cfg, require_verified=False)


if __name__ == "__main__":
    unittest.main()
