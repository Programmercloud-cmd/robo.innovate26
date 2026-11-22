"""Explicit stud-grid geometry for the prescribed 4-ridge x 3-brick stack.

P origin: plate stud (0,0), at the first brick's fully seated bottom datum;
+Z is outward. Returned poses describe BRICK bottom centers, never robot TCPs.
The lattice is nominal geometry: stud coincidence is not verified engagement.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path


NOMINAL_GRID_SOURCE = {
    "url": "https://ldraw.org/article/218.html",
    "description": "LDraw specification: one stud/brick width=20 LDU; approximate 0.4 mm/LDU gives nominal 8 mm pitch. This is a CAD prior, not a measurement of this plate.",
    "nominal_stud_pitch_m": 0.008,
    "physical_pitch_measured": False,
}


class LayoutError(ValueError):
    """Configuration/registration is unresolved or geometrically inconsistent."""


def _integer(value, label, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise LayoutError(f"{label}: integer >= {minimum} required")
    return value


def _positive(value, label):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
        raise LayoutError(f"{label}: positive finite SI value required")
    return float(value)


def _rotate(point, turns):
    x, y = point
    return ((x, y), (-y, x), (-x, -y), (y, -x))[turns % 4]


def _cross(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])


def _hull(points):
    points = sorted(set(points))
    if len(points) <= 1:
        return points
    lower, upper = [], []
    for destination, sequence in ((lower, points), (upper, reversed(points))):
        for point in sequence:
            while len(destination) >= 2 and _cross(destination[-2], destination[-1], point) <= 0:
                destination.pop()
            destination.append(point)
    return lower[:-1] + upper[:-1]


def _inside_hull(point, hull):
    if not hull:
        return False
    if len(hull) == 1:
        return point == hull[0]
    if len(hull) == 2:
        a, b = hull
        return (_cross(a, b, point) == 0 and min(a[0], b[0]) <= point[0] <= max(a[0], b[0])
                and min(a[1], b[1]) <= point[1] <= max(a[1], b[1]))
    return all(_cross(hull[i], hull[(i+1) % len(hull)], point) >= 0 for i in range(len(hull)))


def _center(stack, shape):
    mode = stack.get("center_mode")
    if mode == "plate_center":
        center = [(size-1)/2 for size in shape]
    elif mode == "explicit_stud":
        center = stack.get("center_stud")
    else:
        raise LayoutError("stack.center_mode must explicitly select plate_center or explicit_stud")
    if not isinstance(center, (tuple, list)) or len(center) != 2 or any(
            isinstance(x, bool) or not isinstance(x, (float, int)) or not math.isfinite(x) for x in center):
        raise LayoutError("stack center must be resolved in plate stud coordinates")
    # A 2x4's stud centers have half-integer offsets around its center.
    if any(abs((float(x)-.5)-round(float(x)-.5)) > 1e-9 for x in center):
        raise LayoutError("stack center must be half-integer on both plate stud axes for 90-degree grid registration")
    return tuple(float(x) for x in center)


def generate_layout(config):
    """Return all bricks, potential stud pairs and checked placement dependencies.

    Config keys: baseplate.grid_shape=[columns,rows], stud_pitch_m and optional
    grid_shape_status/stud_pitch_status; stack.center_mode, center_stud (explicit
    mode), layer_count, layer_pitch_m, orientation_quarter_turns and optional
    placement_order ('layer_major' or an exact list of all brick IDs).
    A nominal layout is useful offline while registration remains unverified.
    """
    plate, stack = config["baseplate"], config["stack"]
    shape = plate.get("grid_shape")
    if not isinstance(shape, (tuple, list)) or len(shape) != 2:
        raise LayoutError("baseplate.grid_shape=[columns,rows] required")
    shape = tuple(_integer(x, "plate grid count", 1) for x in shape)
    pitch = _positive(plate.get("stud_pitch_m"), "stud pitch")
    height = _positive(stack.get("layer_pitch_m"), "layer pitch")
    layers = _integer(stack.get("layer_count"), "layer count", 1)
    initial_turn = _integer(stack.get("orientation_quarter_turns", 0), "initial quarter turn") % 4
    center = _center(stack, shape)
    bricks, by_layer = [], []
    for k in range(layers):
        turn = (initial_turn+k) % 4
        current = {}
        for n in range(4):
            for m in range(3):
                local_center = _rotate((4*m-4, 4*n-6), turn)
                brick_center = tuple(center[i]+local_center[i] for i in range(2))
                studs = []
                for v in range(2):
                    for u in range(4):
                        relative = _rotate((4*m-4+u-1.5, 4*n-6+v-.5), turn)
                        grid = tuple(center[i]+relative[i] for i in range(2))
                        if any(abs(x-round(x)) > 1e-9 for x in grid):
                            raise LayoutError("brick stud failed plate-grid parity")
                        grid = tuple(int(round(x)) for x in grid)
                        if any(not 0 <= grid[i] < shape[i] for i in range(2)):
                            raise LayoutError(f"b_{n}_{m}_{k}: stud {grid} lies outside plate grid {shape}")
                        if grid in current:
                            raise LayoutError(f"same-layer stud overlap at {grid}")
                        slot = {"slot": [u, v], "grid": list(grid),
                                "bottom_socket_P_m": [grid[0]*pitch, grid[1]*pitch, k*height],
                                "top_stud_root_P_m": [grid[0]*pitch, grid[1]*pitch, (k+1)*height]}
                        studs.append(slot)
                        current[grid] = (f"b_{n}_{m}_{k}", [u, v])
                matches, unmatched, support_points = [], [], []
                for stud in studs:
                    grid = tuple(stud["grid"])
                    if k == 0:
                        owner, lower_slot = "baseplate", list(grid)
                    elif grid in by_layer[k-1]:
                        owner, lower_slot = by_layer[k-1][grid]
                    else:
                        unmatched.append(stud["slot"])
                        continue
                    support_points.append(grid)
                    matches.append({"upper_socket": stud["slot"], "lower_owner": owner,
                                    "lower_stud_slot": lower_slot, "grid": list(grid),
                                    "engagement_verified": False})
                hull = _hull(support_points)
                inside = _inside_hull(brick_center, hull)
                contributors = sorted({match["lower_owner"] for match in matches})
                support = {"requested_socket_count": 8, "potential_match_count": len(matches),
                           "matches": matches, "unmatched_socket_slots": unmatched,
                           "contributors": contributors, "support_hull_stud": [list(p) for p in hull],
                           "support_affine_rank": 2 if len(hull) >= 3 else max(0, len(hull)-1),
                           "center_in_support_hull": inside,
                           "requires_cantilever_clutch_validation": not inside,
                           "bridges_multiple_lower_bricks": k > 0 and len(contributors) > 1,
                           "mechanical_stability_validated": False,
                           "meaning": "nominal stud coincidences only; no seated/clutch/contact-success assertion"}
                bricks.append({"id": f"b_{n}_{m}_{k}", "index": [n, m, k],
                               "yaw_quarter_turns": turn, "yaw_rad": turn*math.pi/2,
                               "center_stud": list(brick_center),
                               "bottom_center_P_m": [brick_center[0]*pitch, brick_center[1]*pitch, k*height],
                               "studs": studs, "support": support})
        by_layer.append(current)
    canonical = [brick["id"] for brick in bricks]
    order = stack.get("placement_order", "layer_major")
    if order == "layer_major":
        order = canonical
    if not isinstance(order, list) or len(order) != len(canonical) or len(set(order)) != len(order) or set(order) != set(canonical):
        raise LayoutError("placement_order must be layer_major or each configured b_n_m_k exactly once")
    indexed = {brick["id"]: brick for brick in bricks}
    placed = {"baseplate"}
    for identity in order:
        dependencies = set(indexed[identity]["support"]["contributors"])
        if not dependencies <= placed:
            raise LayoutError(f"{identity}: supporting bricks not yet placed: {sorted(dependencies-placed)}")
        placed.add(identity)
    match_counts = Counter(brick["support"]["potential_match_count"] for brick in bricks)
    return {"schema": "lego_requested_stack_layout_v1", "frame": "P: plate stud(0,0), seated brick-bottom datum, +Z outward",
            "pose_semantics": "brick bottom center, NOT end-effector/TCP; held-brick transform still required",
            "grid_shape": list(shape), "stud_pitch_m": pitch, "layer_pitch_m": height,
            "stack_center_stud": list(center), "stack_center_mode": stack["center_mode"],
            "stack_center_P_m": [center[0]*pitch, center[1]*pitch, 0.],
            "initial_orientation_quarter_turns": initial_turn,
            "nominal_layer_footprint_studs": [12, 14], "bricks_per_layer": 12,
            "layer_count": layers, "placement_order": order, "bricks": bricks,
            "summary": {"brick_count": len(bricks), "potential_match_count_histogram": dict(sorted(match_counts.items())),
                        "cantilever_validation_bricks": [b["id"] for b in bricks if b["support"]["requires_cantilever_clutch_validation"]]},
            "provenance": {"nominal_grid_source": NOMINAL_GRID_SOURCE,
                           "grid_shape_status": plate.get("grid_shape_status", "unverified"),
                           "stud_pitch_status": plate.get("stud_pitch_status", "unverified"),
                           "layer_pitch_source": stack.get("layer_pitch_source", "configuration; not independently verified"),
                           "body_dimensions_are_not_grid_pitch": True},
            "training_ready": False, "physical_engagement_validated": False}


def compile_world_poses(config, layout=None, *, require_verified=True):
    """Transform brick geometry only; unresolved registration cannot yield poses.

    baseplate.T_O_plate is a proper rigid 4x4 matrix for the documented P datum.
    Verified mode additionally requires registration_status='verified' and both
    grid_shape_status and stud_pitch_status in {'verified','measured'}.
    Preview mode is explicit and never implies physical readiness.
    """
    plate = config["baseplate"]
    transform = plate.get("T_O_plate")
    if not isinstance(transform, list) or len(transform) != 4 or any(not isinstance(row, list) or len(row) != 4 for row in transform):
        raise LayoutError("baseplate.T_O_plate unresolved; no invented world coordinates")
    if any(isinstance(x, bool) or not isinstance(x, (float, int)) or not math.isfinite(x) for row in transform for x in row):
        raise LayoutError("finite rigid T_O_plate required")
    if any(abs(transform[3][i]-expected) > 1e-9 for i, expected in enumerate((0, 0, 0, 1))):
        raise LayoutError("invalid rigid-transform last row")
    rotation = [row[:3] for row in transform[:3]]
    if any(abs(sum(rotation[i][j]*rotation[k][j] for j in range(3))-(i == k)) > 1e-7 for i in range(3) for k in range(3)):
        raise LayoutError("T_O_plate rotation must be orthonormal, without scale/shear")
    determinant = sum(rotation[0][i]*(rotation[1][(i+1)%3]*rotation[2][(i+2)%3]-rotation[1][(i+2)%3]*rotation[2][(i+1)%3]) for i in range(3))
    if determinant < .999999:
        raise LayoutError("T_O_plate rotation must be proper")
    verified = (plate.get("registration_status") == "verified" and all(
        plate.get(name) in ("verified", "measured") for name in ("grid_shape_status", "stud_pitch_status")))
    if require_verified and not verified:
        raise LayoutError("world registration/grid still unverified; preview must be explicit")
    expected = generate_layout(config)
    if layout is not None and (layout.get("bricks") != expected["bricks"] or layout.get("placement_order") != expected["placement_order"]):
        raise LayoutError("provided layout is stale or does not match current geometry/configuration")
    layout = expected
    poses = []
    for brick in layout["bricks"]:
        quarter = brick["yaw_quarter_turns"]
        c, s = ((1,0),(0,1),(-1,0),(0,-1))[quarter]
        local_rotation = ((c,-s,0),(s,c,0),(0,0,1))
        position = [transform[i][3]+sum(rotation[i][j]*brick["bottom_center_P_m"][j] for j in range(3)) for i in range(3)]
        world_rotation = [[sum(rotation[i][j]*local_rotation[j][k] for j in range(3)) for k in range(3)] for i in range(3)]
        poses.append({"id": brick["id"], "brick_bottom_center_O_m": position, "R_O_brick": world_rotation})
    return {"poses": poses, "registered_geometry_verified": verified, "preview_only": not require_verified,
            "tool_poses": None, "tool_constraint": "EE must remain vertical in world; brick rotation is not a tool rotation command",
            "physical_engagement_validated": False, "training_ready": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--world", action="store_true", help="require verified physical plate registration")
    args = parser.parse_args()
    try:
        config = json.loads(args.config.read_text())
        report = generate_layout(config)
        if args.world:
            report["world"] = compile_world_poses(config, report)
    except (KeyError, TypeError, ValueError, OSError) as error:
        parser.exit(2, f"Recoverable stop: {error}\n")
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")


if __name__ == "__main__":
    main()
