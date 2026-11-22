"""Photograph registration candidate, not physical metrology or camera calibration.

Run with /usr/bin/python3 -s for the installed OpenCV/NumPy ABI. Uses the known
ChArUco pattern and a supplied plate ROI; no robot, camera capture or hardware IO.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def rigid_fit(local, observed):
    """SO(2) rigid least squares; never absorbs apparent scale into stud pitch."""
    u, _, vh = np.linalg.svd((local-local.mean(0)).T @ (observed-observed.mean(0)))
    rotation = vh.T @ u.T
    if np.linalg.det(rotation) < 0:
        vh[-1] *= -1
        rotation = vh.T @ u.T
    translation = observed.mean(0) - rotation @ local.mean(0)
    error = local @ rotation.T + translation - observed
    return rotation, translation, float(np.sqrt(np.mean(np.sum(error**2, axis=1))))


def analyze(config, repository):
    import cv2
    spec = config["registration"]
    photo = repository/spec["photo"]
    image = cv2.imread(str(photo))
    if image is None:
        raise ValueError("photo unavailable")
    if [image.shape[1],image.shape[0]] != spec["image_size"]:
        raise ValueError("image hints belong to a different image resolution")
    board_spec = spec["board"]
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, board_spec["dictionary"]))
    board = cv2.aruco.CharucoBoard_create(*board_spec["squares_xy"], board_spec["square_length_m"], board_spec["marker_length_m"], dictionary)
    corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
    if ids is None:
        raise ValueError("no board markers; registration cannot proceed")
    count, points, point_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, image, board)
    if count < 8:
        raise ValueError("insufficient board corners")
    object_xy = board.chessboardCorners[point_ids.flatten(), :2]
    H, _ = cv2.findHomography(object_xy, points.reshape(-1, 2), 0)
    projected = cv2.perspectiveTransform(object_xy.reshape(-1,1,2), H).reshape(-1,2)
    board_rms = float(np.sqrt(np.mean(np.sum((projected-points.reshape(-1,2))**2,axis=1))))
    inverse = np.linalg.inv(H)
    x0,y0,x1,y1 = spec["plate_roi_pixels"]
    gray = cv2.cvtColor(cv2.resize(image[y0:y1,x0:x1], None, fx=.5, fy=.5), cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,15,param1=80,param2=14,minRadius=5,maxRadius=11)
    if circles is None:
        raise ValueError("plate ROI has insufficient circular features")
    pixels = (circles[0,:,:2]*2+[x0,y0]).astype(np.float32)
    shape = config["baseplate"]["grid_shape"]
    grid_corners = np.array([[0,0],[shape[0]-1,0],[shape[0]-1,shape[1]-1],[0,shape[1]-1]],np.float32)
    grid_H = cv2.getPerspectiveTransform(grid_corners,np.asarray(spec["first_stud_corner_hints_pixels"],np.float32))
    for _ in range(5):
        grid_xy = cv2.perspectiveTransform(pixels.reshape(-1,1,2),np.linalg.inv(grid_H)).reshape(-1,2)
        indices = np.rint(grid_xy).astype(int)
        valid = (np.linalg.norm(grid_xy-indices,axis=1)<.38) & (indices.min(1)>=0) & (indices[:,0]<shape[0]) & (indices[:,1]<shape[1])
        unique = {}
        for index in np.where(valid)[0]:
            key = tuple(indices[index])
            residual = float(np.linalg.norm(grid_xy[index]-indices[index]))
            if key not in unique or residual < unique[key][0]:
                unique[key] = residual,index
        selected = [item[1] for item in unique.values()]
        if len(selected)<.8*shape[0]*shape[1]:
            raise ValueError("configured grid shape not supported by enough photo features")
        grid_H,_ = cv2.findHomography(indices[selected].astype(np.float32),pixels[selected],0)
    pixels = pixels[selected]
    indices = indices[selected]
    observed = cv2.perspectiveTransform(pixels.reshape(-1,1,2),inverse).reshape(-1,2)
    pitch = config["baseplate"]["stud_pitch_m"]
    rotation,translation,grid_rms = rigid_fit(indices*pitch, observed)
    # Tape edge is detected outside the plate ROI so stud shadows cannot be
    # mistaken for the black stripe. These are image-analysis bounds only.
    gray_full = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    tape = []
    scan = spec["tape_scan_pixels"]
    y_start,y_stop = scan["y_range"]
    for x in list(range(*scan["left_x_range"],10))+list(range(*scan["right_x_range"],10)):
        dark = np.where(gray_full[y_start:y_stop,x]<60)[0]
        if len(dark):
            tape.append((x,y_start+int(dark[0])))
    tape = np.asarray(tape,float)
    if len(tape)<10:
        raise ValueError("tape edge unavailable; do not infer another edge")
    line = np.linalg.lstsq(np.c_[tape[:,0],np.ones(len(tape))],tape[:,1],rcond=None)[0]
    residual = tape[:,1] - np.c_[tape[:,0],np.ones(len(tape))] @ line
    good = abs(residual-np.median(residual))<10
    line = np.linalg.lstsq(np.c_[tape[good,0],np.ones(sum(good))],tape[good,1],rcond=None)[0]
    tape_pixels = np.asarray([[[x,line[0]*x+line[1]]] for x in [x0+50,x1-110]],np.float32)
    tape_board = cv2.perspectiveTransform(tape_pixels,inverse).reshape(-1,2)
    # Candidate enforces the user's centerline and tape-edge constraints. The
    # unconstrained rigid fit remains evidence of their disagreement/uncertainty.
    center_x = board_spec["squares_xy"][0]*board_spec["square_length_m"]/2
    tape_y = float(tape_board[:,1].mean())
    T_B_P = np.eye(4)
    T_B_P[:2,3] = [center_x-(shape[0]-1)*pitch/2,tape_y-(shape[1]-.5)*pitch]
    prior_path = repository/spec["old_scene_registration"]
    T_O_B = np.asarray(json.loads(prior_path.read_text())["T_base_board"])
    constrained_prediction = indices*pitch+T_B_P[:2,3]
    return {
        "status":"photo_candidate_only", "physical_registration_verified":False,
        "image_size":[image.shape[1],image.shape[0]],"marker_ids":ids.flatten().tolist(),
        "charuco_corner_count":int(count),"board_rms_px":board_rms,
        "H_board_to_image":H.tolist(),"H_grid_to_image":grid_H.tolist(),
        "configured_grid_shape":shape,"unique_matched_studs":len(indices),
        "matched_indices":indices.tolist(),"matched_pixels":pixels.tolist(),
        "unconstrained_rigid_grid_rms_m":grid_rms,
        "unconstrained_grid_rotation_B":rotation.tolist(),"unconstrained_grid_origin_B_m":translation.tolist(),
        "constraint_grid_rms_m":float(np.sqrt(np.mean(np.sum((constrained_prediction-observed)**2,axis=1)))),
        "tape_edge_pixels":tape_pixels.reshape(-1,2).tolist(),"tape_edge_board_m":tape_board.tolist(),
        "resolved_photo_axes":"plate +X follows board +X (photo left); +Y follows board +Y (photo down); +Y plate edge faces board-side tape edge",
        "T_B_P_display_proxy":T_B_P.tolist(),"T_O_P_display_proxy":(T_O_B@T_B_P).tolist(),
        "physical_T_O_plate":None,"plate_thickness_m":None,"seating_datum_height_m":None,
        "uncertainty_m":None,
        "limitations":["Source-camera identity/lens distortion/intrinsics unknown; board-plane extrapolation is not metrology","Baseplate height differs from paper plane by unmeasured thickness/support","Nominal stud pitch and prior board-to-base calibration are not independently validated","Photo constraints resolve candidate axes/edge, not robot or wrist-camera mounting continuity","Configured 32x32 shape is photo-supported, not physically surveyed"],
        "source_sha256":{str(photo):hashlib.sha256(photo.read_bytes()).hexdigest(),str(prior_path):hashlib.sha256(prior_path.read_bytes()).hexdigest()}
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,default=Path(__file__).with_name("config.json"))
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    config=json.loads(args.config.read_text())
    repository=(args.config.resolve().parent/config["repository_root"]).resolve()
    report=analyze(config,repository)
    args.output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({key:report[key] for key in ["status","charuco_corner_count","unique_matched_studs","unconstrained_rigid_grid_rms_m","constraint_grid_rms_m"]}))


if __name__=="__main__":
    main()
