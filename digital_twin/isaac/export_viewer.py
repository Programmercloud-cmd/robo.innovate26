"""Export USD visual triangles for an optional WebGL monitor of PhysX state."""
import argparse
import json
from pathlib import Path


def export(stage, link_names, root, output):
    from pxr import Gf, Usd, UsdGeom, UsdShade
    meshes = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if prim.GetTypeName() != "Mesh" or "/collisions" in path.lower():
            continue
        if UsdGeom.Imageable(prim).ComputeVisibility() == "invisible":
            continue
        link = next((i for i, name in enumerate(link_names)
                     if path.startswith(root + "/" + name + "/")), -1)
        transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        if link >= 0:
            transform = transform * UsdGeom.Xformable(stage.GetPrimAtPath(root + "/" + link_names[link])).ComputeLocalToWorldTransform(0).GetInverse()
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        if points is None or counts is None or indices is None:
            continue
        holes = set(mesh.GetHoleIndicesAttr().Get() or [])
        color = [.82, .83, .84]
        display_color = mesh.GetDisplayColorAttr().Get()
        if display_color:
            color = list(display_color[0])
        if any(v in path for v in ("C920", "c920", "UpdatedTipSTL", "D435_mount")):
            color = [.025, .028, .032]
        if "Screw" in path:
            color = [.3, .31, .32]
        subset_colors = {}
        for subset in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            subset_color = [.02, .022, .025] if str(subset.GetPath()).endswith("material_1") else color
            for face in subset.GetIndicesAttr().Get() or []:
                subset_colors[int(face)] = subset_color
        vertices = [round(float(v), 7) for p in points for v in transform.Transform(Gf.Vec3d(p))]
        colors = color * len(points)
        triangles = []
        offset = 0
        for face, count in enumerate(counts):
            face_indices = indices[offset:offset+count]
            offset += count
            if face in holes:
                continue
            for j in range(1, count-1):
                for index in (face_indices[0], face_indices[j], face_indices[j+1]):
                    triangles.append(int(index))
                    colors[index*3:index*3+3] = subset_colors.get(face, color)
        meshes.append({"name": path, "link": link, "vertices": vertices, "indices": triangles, "colors": colors})
    import gzip
    payload = json.dumps({"link_names": link_names, "meshes": meshes}, separators=(",", ":")).encode()
    (output / "visuals.json.gz").write_bytes(gzip.compress(payload))
    import shutil
    shutil.copy2(Path(__file__).with_name("viewer.html"), output / "index.html")
    shutil.copy2(Path(__file__).with_name("brick_view.js"), output / "brick_view.js")
    return {"meshes": len(meshes), "triangles": sum(len(m["indices"])//3 for m in meshes),
            "purpose": "WebGL presentation of exported USD visual meshes driven by actual PhysX link state; not an Isaac RTX render"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "enable_crashreporter": False}, experience=str(Path(__file__).with_name("physics_only.kit")))
    from pxr import Usd
    stage = Usd.Stage.Open(str(args.output / "digital_twin.usda"))
    meta = json.loads((args.output / "manifest.json").read_text())
    print(export(stage, meta["link_names"], "/World/envs/env_0/Robot", args.output))
    app.close(wait_for_replicator=False)
