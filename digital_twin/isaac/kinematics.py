"""Independent FK from the URDF exported by the physical FR3."""
import math
import xml.etree.ElementTree as ET
import numpy as np


def rotation(axis, angle):
    v = np.asarray(axis, dtype=float)
    v = v / np.linalg.norm(v)
    x, y, z = v
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew@skew)


def pose_xyzw(p):
    transform = np.eye(4)
    transform[:3, 3] = p[:3]
    x, y, z, w = p[3:]
    norm = x*x + y*y + z*z + w*w
    transform[:3, :3] = np.array([
        [1-2*(y*y+z*z)/norm, 2*(x*y-z*w)/norm, 2*(x*z+y*w)/norm],
        [2*(x*y+z*w)/norm, 1-2*(x*x+z*z)/norm, 2*(y*z-x*w)/norm],
        [2*(x*z-y*w)/norm, 2*(y*z+x*w)/norm, 1-2*(x*x+y*y)/norm]])
    return transform


def difference(a, b):
    # Robot state matrices are float32 and may be slightly nonorthogonal.
    # Compare their nearest proper rotations, rather than interpreting rounding
    # in the matrix trace as a physical angular discrepancy.
    def nearest_rotation(matrix):
        u, _, vt = np.linalg.svd(matrix)
        sign = np.linalg.det(u@vt)
        return u@np.diag([1., 1., sign])@vt
    ra, rb = nearest_rotation(a[:3, :3]), nearest_rotation(b[:3, :3])
    return {"position_m": float(np.linalg.norm(a[:3, 3]-b[:3, 3])),
            "rotation_rad": float(math.acos(np.clip((np.trace(ra.T@rb)-1)/2, -1., 1.)))}


class Model:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.chain = []
        for i in range(1, 9):
            joint = root.find(f"joint[@name='joint{i}']")
            origin = joint.find("origin")
            t = np.eye(4)
            t[:3, 3] = [float(v) for v in origin.get("xyz").split()]
            r, p, y = [float(v) for v in origin.get("rpy").split()]
            t[:3, :3] = rotation([0, 0, 1], y)@rotation([0, 1, 0], p)@rotation([1, 0, 0], r)
            axis = [float(v) for v in joint.find("axis").get("xyz").split()] if i < 8 else None
            self.chain.append((t, axis))

    def flange(self, q):
        result = np.eye(4)
        for i, (origin, axis) in enumerate(self.chain):
            result = result@origin
            if axis is not None:
                t = np.eye(4)
                t[:3, :3] = rotation(axis, q[i])
                result = result@t
        return result
