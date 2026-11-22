"""Fixed, taught wrist-image regions for four-brick retention verification.

At grasp height each target brick fills its own near-field region. After a
measured45mm+ lift, unpicked bricks recede out of these regions. This was checked
against actual open-hand grasp-height and unpicked raised images; it is not a
general detector and does not infer robot poses from pixels.
"""
import cv2
import numpy as np


def evidence(image,profile):
    if list(image.shape[1::-1])!=profile['image_size']:raise ValueError('Near-field camera resolution changed')
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);bricks=[]
    for index,(x0,y0,x1,y1) in enumerate(profile['brick_regions_px']):
        p=hsv[y0:y1,x0:x1]
        colored=(p[:,:,1]>35)&(p[:,:,2]>35)&((p[:,:,0]<32)|(p[:,:,0]>170))
        fraction=float(colored.mean())
        bricks.append(dict(brick=index,colored_fraction=fraction,present=fraction>=profile['minimum_colored_fraction']))
    x0,y0,x1,y1=profile['separation_region_px'];p=hsv[y0:y1,x0:x1]
    white=(p[:,:,1]<45)&(p[:,:,2]>80);rows=white.mean(axis=1)>.85
    longest=run=0
    for value in rows:run=run+1 if value else 0;longest=max(longest,run)
    return dict(all_four_present=all(b['present'] for b in bricks),bricks=bricks,
                white_separation_rows=longest,separation_visible=longest>=profile['minimum_separation_rows'])
