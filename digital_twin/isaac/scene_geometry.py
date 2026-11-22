"""Author explicitly supplied, registered fixed geometry in robot-base coordinates.

No image coordinates or guessed tabletop origin are embedded in this module.
"""
import json
from pathlib import Path
from pxr import Gf, Sdf, UsdGeom, UsdShade


def author_registered_scene(stage, registration):
    root = "/World/RegisteredScene"
    frame = UsdGeom.Xform.Define(stage, root)
    matrix = registration["T_base_board"]
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError("explicit 4x4 T_base_board required")
    # USD/Gf uses row vectors; calibration uses column vectors.
    frame.AddTransformOp().Set(Gf.Matrix4d(*[matrix[j][i] for i in range(4) for j in range(4)]))
    frame.GetPrim().CreateAttribute("registrationProvenance", Sdf.ValueTypeNames.String).Set(json.dumps(registration.get("provenance",registration.get("registration", {}))))
    def polygon(name, xy, color, z=0.):
        if len(xy) < 3:
            raise ValueError("polygon requires at least 3 points")
        mesh = UsdGeom.Mesh.Define(stage, root + "/" + name)
        mesh.CreatePointsAttr([Gf.Vec3f(float(x), float(y), float(z)) for x, y in xy])
        mesh.CreateFaceVertexCountsAttr([len(xy)])
        mesh.CreateFaceVertexIndicesAttr(list(range(len(xy))))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        material = UsdShade.Material.Define(stage, root + "/Materials/" + name)
        shader = UsdShade.Shader.Define(stage, str(material.GetPath()) + "/Surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.8)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
        return mesh
    geometry = registration.get("geometry_board_m", {})
    if geometry.get("table_polygon_xy"):
        polygon("Tabletop", geometry["table_polygon_xy"], [.88, .87, .83], -.0002)
    for i, tape in enumerate(geometry.get("tape_polygons_xy", [])):
        polygon(tape.get("name", f"Tape_{i}"), tape["points"], [.025, .025, .026], .0001)
    for i, item in enumerate(geometry.get("additional_polygons_xy", [])):
        polygon(item.get("name", f"Feature_{i}"), item["points"], item.get("color", [.8, .8, .8]), item.get("z_m", .00015))
    board = registration.get("board")
    if board:
        pattern = json.loads(Path(__file__).with_name("board_pattern.json").read_text())
        width, height = pattern["width_m"], pattern["height_m"]
        if any(abs(a-b*board["square_length_m"]) > 1e-7 for a,b in zip((width,height),board["squares_xy"])):
            raise ValueError("board visual extent differs from supplied calibration")
        polygon("BoardPaper", [[0,0],[width,0],[width,height],[0,height]], [.97,.97,.95], .0003)
        points, indices = [], []
        for x0,y0,x1,y1 in pattern["black_rectangles_xyxy"]:
            base = len(points)
            points.extend([Gf.Vec3f(x0,y0,.0004),Gf.Vec3f(x1,y0,.0004),Gf.Vec3f(x1,y1,.0004),Gf.Vec3f(x0,y1,.0004)])
            indices.extend([base,base+1,base+2,base+3])
        mesh = UsdGeom.Mesh.Define(stage, root+"/BoardNominalPrintedPattern")
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([4]*(len(indices)//4))
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDisplayColorAttr([Gf.Vec3f(.015,.015,.015)])
        mesh.GetPrim().CreateAttribute("patternStatus",Sdf.ValueTypeNames.String).Set(pattern["status"])
    return {"status": registration.get("status", "supplied registration; review provenance"),
            "T_base_board": matrix, "provenance": registration.get("provenance",registration.get("registration")),
            "notes": registration.get("notes"),
            "table_polygon": bool(geometry.get("table_polygon_xy")),
            "tape_polygon_count": len(geometry.get("tape_polygons_xy", [])),
            "scope": "static visual surfaces; no inferred collision/contact calibration",
            "surface_offsets": "0.1-0.2mm drawing offsets prevent z-fighting; not measured physical tape thickness"}
