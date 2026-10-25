"""Tiny no-capture geometry/frame/serialization regressions; no simulation."""
import json
import math
import numpy as np
from evaluate import polygon_iou,relative_pose,initial_matches
from contact_helpers import rotation,angle_between,classify_point

def main():
    square=[[0,0],[10,0],[10,10],[0,10]]
    assert polygon_iou(square,square)==1
    assert initial_matches(square,[{'id':'hidden','layer':7,'image_polygon':square},{'id':'top','layer':9,'image_polygon':square}])==[{'id':'top','iou':1.,'layer':9}]
    assert polygon_iou(square,[[20,0],[30,0],[30,10],[20,10]])==0
    tcp=[1,2,3,1,0,0,0];body=[1.001,2.002,3.003,1,0,0,0]
    p,r=relative_pose(tcp,body)
    assert np.allclose(p,[.001,.002,.003]) and np.array_equal(r,np.eye(3))
    assert angle_between([1,0,0,0],[-1,0,0,0])==0
    q=[math.sqrt(.5),0,0,math.sqrt(.5)]
    assert np.allclose(rotation(q)@[1,0,0],[0,1,0],atol=1e-15)
    protocol={'geometry':{'pad_distance_allowance_m':50e-6,'white_extra_clearance_m':.0001,'side_contact':{'minimum_outward_dot_grip_normal':.9}}}
    vertices=[[-1,0,-1],[1,0,-1],[1,0,1],[-1,0,1]]
    colliders=[{'enabled':True,'role':role,'points_finger_m':(np.asarray(vertices)+[0,y,0]).tolist(),'triangle_indices':[[0,1,2],[0,2,3]]} for role,y in [('body',0),('white',1),('head',2)]]
    result=classify_point([0,0,0],0,[0,0,0,1,0,0,0],colliders,protocol,outward_world=[0,-1,0],flat_face_y_m=0,expected_grip_normal_finger=[0,-1,0])
    assert result['uniquely_pad_compatible']
    wrong=classify_point([0,0,0],0,[0,0,0,1,0,0,0],colliders,protocol,outward_world=[0,0,1],flat_face_y_m=0,expected_grip_normal_finger=[0,-1,0])
    assert not wrong['uniquely_pad_compatible']
    json.dumps({'pass':np.bool_(True)},default=lambda x:x.item(),allow_nan=False)
    print('Synthetic polygon/frame/all-point normal-domain/serialization checks PASS')
if __name__=='__main__':main()
