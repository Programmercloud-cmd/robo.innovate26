"""Fixed-template edge servo: no scene reconstruction or general detector."""
import json
import math
import cv2
import numpy as np


class Uncertain(RuntimeError):pass


def reference_key(layer,stage):
    if stage=='align' and layer['index']==0:return 'align_baseplate'
    return f"{stage}_layer_{layer['index']%2}"


def angle_delta(a,b):return (a-b+math.pi/2)%math.pi-math.pi/2


def fit_edge(image,segment,settings):
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) if image.ndim==3 else image
    polarity=segment.get('polarity',1) if isinstance(segment,dict) else 1
    if isinstance(segment,dict):segment=segment['segment']
    if polarity not in (-1,1):raise Uncertain('Edge polarity must be +1 or -1')
    a,b=np.asarray(segment,dtype=float);tangent=b-a;length=np.linalg.norm(tangent)
    if length<20:raise Uncertain('Reference edge is too short')
    tangent/=length;normal=np.array([-tangent[1],tangent[0]])
    radius=settings['search_radius_px'];along=np.linspace(.08,.92,48)*length
    offsets=np.arange(-radius,radius+1);grid=a+along[:,None,None]*tangent+offsets[None,:,None]*normal
    if np.any(grid[:,:,0]<1) or np.any(grid[:,:,0]>=gray.shape[1]-1) or np.any(grid[:,:,1]<1) or np.any(grid[:,:,1]>=gray.shape[0]-1):raise Uncertain('Edge sampling leaves image')
    strip=cv2.remap(gray,grid[:,:,0].astype(np.float32),grid[:,:,1].astype(np.float32),cv2.INTER_LINEAR).astype(float)
    # Track a specified contrast direction, not alternating sides of a thick
    # edge as subpixel rasterization changes their relative peak strengths.
    gradient=np.maximum(0.,polarity*np.diff(strip,axis=1));best=gradient.argmax(1);strength=gradient[np.arange(len(along)),best]
    valid=strength>20
    if valid.mean()<settings['minimum_edge_coverage']:raise Uncertain('Edge occluded or low contrast')
    # Intensity-gradient centroid gives subpixel position. Integer peak bins
    # otherwise report zero yaw before a long layer is actually aligned.
    near=np.abs(np.arange(gradient.shape[1])[None,:]-best[:,None])<=2
    weights=gradient*near
    shifts=(weights*(offsets[:-1]+.5)[None,:]).sum(1)/np.maximum(weights.sum(1),1e-12)
    coef=np.polyfit(along[valid]-length/2,shifts[valid],1)
    residual=np.abs(shifts-np.polyval(coef,along-length/2));valid &= residual<settings['maximum_edge_residual_px']
    if valid.mean()<settings['minimum_edge_coverage']:raise Uncertain('Edge fit is inconsistent')
    coef=np.polyfit(along[valid]-length/2,shifts[valid],1)
    if abs(coef[1])>=radius-2:raise Uncertain('Edge reached search boundary')
    endpoints=np.array([a,b])+((np.array([-length/2,length/2])*coef[0]+coef[1])[:,None])*normal
    return dict(segment=endpoints.tolist(),coverage=float(valid.mean()),offset_px=float(coef[1]),residual_px=float(residual[valid].mean()))


def intersection(a,b):
    p,q=np.asarray(a);r,s=np.asarray(b);matrix=np.column_stack([q-p,r-s])
    if abs(np.linalg.det(matrix))<.25*np.linalg.norm(q-p)*np.linalg.norm(s-r):raise Uncertain('Need two nonparallel visible edges')
    t=np.linalg.solve(matrix,r-p)[0];return p+t*(q-p)


def feature(image,reference,settings):
    edges={key:fit_edge(image,segment,settings) for key,segment in reference['edges'].items()}
    for key in ('held_u','held_v'):
        if key in edges and abs(edges[key]['offset_px'])>settings['maximum_grasp_edge_shift_px']:raise Uncertain('Held layer moved relative to wrist camera')
    u=edges['receiver_u']['segment'];v=edges['receiver_v']['segment'];corner=intersection(u,v)
    vector=np.subtract(u[1],u[0]);angle=math.atan2(vector[1],vector[0])
    return np.array([corner[0],corner[1],angle]),edges


def correction(image,reference,settings):
    value,edges=feature(image,reference,settings);target=np.array(reference['target_feature'])
    error=value-target;error[2]=angle_delta(value[2],target[2]);J=np.array(reference['image_jacobian'])
    if J.shape!=(3,3) or not np.isfinite(J).all():raise Uncertain('Missing alignment image Jacobian')
    normalized=np.diag([1.,1.,reference.get('angle_feature_scale_px',150.)])@J@np.diag([.001,.001,.01])
    if np.linalg.cond(normalized)>settings['maximum_calibration_condition']:raise Uncertain('Ill-conditioned image calibration')
    delta=-np.linalg.solve(J,error)
    if not np.isfinite(delta).all():raise Uncertain('Nonfinite visual correction')
    return delta,dict(feature=value.tolist(),error=error.tolist(),edges=edges)


def overlay(image,evidence):
    image=image.copy()
    for label,edge in evidence['edges'].items():
        a,b=np.round(edge['segment']).astype(int);cv2.line(image,tuple(a),tuple(b),(0,220,100),2)
        cv2.putText(image,label,tuple(a),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,220,100),1)
    return image


def fit_calibration(samples,target_feature):
    """Offline fit from actual image features and known local robot offsets."""
    action=np.asarray([s['delta_xy_yaw'] for s in samples]);features=np.asarray([s['feature'] for s in samples])
    if action.ndim!=2 or action.shape[1]!=3 or len(action)<7:raise ValueError('Need neutral and +/-X, +/-Y, +/-yaw samples')
    design=np.c_[np.ones(len(action)),action]
    if np.linalg.matrix_rank(design)!=4:raise ValueError('Calibration does not span X/Y/yaw')
    features[:,2]=np.unwrap(features[:,2]);coeff=np.linalg.lstsq(design,features,rcond=None)[0]
    residual=features-design@coeff
    return dict(image_jacobian=coeff[1:].T.tolist(),target_feature=target_feature,
                fit_rms=[float(x) for x in np.sqrt(np.mean(residual**2,axis=0))],physical_validation=False)
