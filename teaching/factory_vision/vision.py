"""Known-layout four-brick row evidence. No robot imports or motion commands."""
import math
import subprocess
import time
from pathlib import Path
import cv2
import numpy as np


class Uncertain(RuntimeError):
    pass


class Camera:
    def __init__(self, config):
        self.config=config
        device=config['device']
        if not Path(device).exists():raise Uncertain('Configured wrist camera is disconnected')
        self.cap=cv2.VideoCapture(device,cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,config['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,config['height'])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
        if not self.cap.isOpened():raise Uncertain('Cannot open wrist Logitech camera')
        self.first_frame=True
        self.previous_observation=None
        subprocess.run(['v4l2-ctl','-d',device,'--set-ctrl',
            f"focus_automatic_continuous=0,focus_absolute={config['focus']}"],check=True,capture_output=True)
        if config.get('controls'):
            subprocess.run(['v4l2-ctl','-d',device,'--set-ctrl',
                ','.join(f'{key}={value}' for key,value in config['controls'].items())],check=True,capture_output=True)

    def frame(self):
        for _ in range(self.config['discard_frames'] if self.first_frame else 1):
            for attempt in range(3):
                try:ok,im=self.cap.read()
                except cv2.error:ok,im=False,None
                if ok and im is not None:break
            if not ok:raise Uncertain('Wrist camera frame failed')
        if im.shape[:2]!=(self.config['height'],self.config['width']):
            raise Uncertain('Unexpected camera resolution')
        self.first_frame=False
        return im

    def close(self):self.cap.release()


def fit_line(points, weights, tolerance):
    points=np.asarray(points,dtype=float);weights=np.asarray(weights,dtype=float)
    best=None
    for i in range(0,len(points),2):
        a,b=points[i:i+2]
        if abs(b[0]-a[0])<1:continue
        slope=(b[1]-a[1])/(b[0]-a[0]);intercept=a[1]-slope*a[0]
        inside=np.abs(points[:,1]-slope*points[:,0]-intercept)<tolerance
        score=weights[inside].sum()
        if best is None or score>best[0]:best=(score,inside)
    if best is None or best[1].sum()<2:raise Uncertain('Insufficient boundary line evidence')
    inside=best[1]
    a,b=np.polyfit(points[inside,0],points[inside,1],1,w=np.sqrt(weights[inside]))
    return float(a),float(b),points[inside]


def detect(image, config):
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    x0,y0,x1,y1=config['stack_roi_px']
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask=((hsv[:,:,1]>80)&(hsv[:,:,2]>35)&((hsv[:,:,0]<32)|(hsv[:,:,0]>170))).astype(np.uint8)*255
    outside=np.zeros_like(mask);outside[y0:y1,x0:x1]=mask[y0:y1,x0:x1]
    outside=cv2.morphologyEx(outside,cv2.MORPH_CLOSE,np.ones((9,9),np.uint8))
    count,labels,stats,_=cv2.connectedComponentsWithStats(outside)
    if count<2:raise Uncertain('No colored source stack in the expected region')
    index=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA]));mask=(labels==index).astype(np.uint8)
    if stats[index,cv2.CC_STAT_AREA]<80000:raise Uncertain('Source stack occupies too little image area')
    segments=cv2.createLineSegmentDetector().detect(gray[y0:y1,x0:x1])[0]
    if segments is None:raise Uncertain('No boundary lines')
    segments=segments+np.array([x0,y0,x0,y0],np.float32).reshape(1,1,4)
    boundaries=[];coverage=[]
    for expected_y in config['boundary_y_px']:
        points=[];weights=[]
        for line in segments:
            ax,ay,bx,by=map(float,line.ravel());length=math.hypot(bx-ax,by-ay)
            angle=(math.degrees(math.atan2(by-ay,bx-ax))+90)%180-90
            if length<config['minimum_line_length_px'] or abs(angle)>config['maximum_line_angle_deg']:continue
            mx,my=(ax+bx)/2,(ay+by)/2
            if not x0<mx<x1 or abs(my-expected_y)>config['boundary_search_px']:continue
            ix,iy=round(mx),round(my)
            if not mask[max(0,iy-4):iy+5,max(0,ix-4):ix+5].any():continue
            points.extend([[ax,ay],[bx,by]]);weights.extend([length,length])
        a,b,inliers=fit_line(points,weights,config['line_inlier_px'])
        span=float(np.ptp(inliers[:,0]));boundaries.append((a,b));coverage.append(span)
    top,bottom=boundaries
    middle_y=sum(a*640+b for a,b in boundaries)/2
    if not 200<middle_y<420:raise Uncertain('Row center outside expected image region')
    side_points=[[],[]]
    for y in range(round(middle_y-90),round(middle_y+90),3):
        xs=np.flatnonzero(mask[y])
        if len(xs)<450:continue
        if xs[0]<=x0+3 or xs[-1]>=x1-3:continue
        side_points[0].append([y,float(xs[0])]);side_points[1].append([y,float(xs[-1])])
    sides=[]
    for values in side_points:
        if len(values)<20:raise Uncertain('Both outer brick edges must be visible')
        values=np.array(values);a,b=np.polyfit(values[:,0],values[:,1],1)
        residual=np.abs(values[:,1]-a*values[:,0]-b);good=residual<max(4,float(np.median(residual))*3)
        a,b=np.polyfit(values[good,0],values[good,1],1);sides.append((float(a),float(b)))
    corners=[]
    for boundary,side in [(top,sides[0]),(top,sides[1]),(bottom,sides[1]),(bottom,sides[0])]:
        a,b=boundary;c,d=side;x=(c*b+d)/(1-c*a);corners.append([x,a*x+b])
    corners=np.array(corners,np.float32)
    angle=math.atan((top[0]+bottom[0])/2)
    return validate_row(image,corners,config,coverage,angle)


def validate_row(image,corners,config,coverage,angle):
    x0,y0,x1,y1=config['stack_roi_px']
    width=float(np.linalg.norm(corners[1]-corners[0]));height=float(np.linalg.norm(corners[3]-corners[0]))
    if not 1.7<width/height<2.7:raise Uncertain('Observed row does not have the four-brick footprint aspect ratio')
    # Wrist perspective makes the far/near edges different lengths. Placement
    # coverage must use each edge's own length, not the longer top edge twice.
    boundary_widths=[width,float(np.linalg.norm(corners[2]-corners[3]))] if config.get('perspective_boundary_coverage') else [width,width]
    normalized_coverage=[v/w for v,w in zip(coverage,boundary_widths)]
    partial_boundary=min(normalized_coverage)<config['minimum_boundary_coverage']
    if partial_boundary and (min(normalized_coverage)<.22 or max(normalized_coverage)<.75):
        raise Uncertain('Row boundary support is too sparse')
    if np.any(corners[:,0]<x0+3) or np.any(corners[:,0]>x1-3):raise Uncertain('Clipped row footprint')
    homography=cv2.getPerspectiveTransform(corners,np.array([[0,0],[256,0],[256,128],[0,128]],np.float32))
    rectified=cv2.warpPerspective(image,homography,(256,128))
    rect_gray=cv2.cvtColor(rectified,cv2.COLOR_BGR2GRAY)
    rect_gray=cv2.createCLAHE(clipLimit=2,tileGridSize=(8,4)).apply(rect_gray)
    circles=cv2.HoughCircles(cv2.GaussianBlur(rect_gray,(5,5),.8),cv2.HOUGH_GRADIENT,
        dp=1,minDist=18,param1=65,param2=12,minRadius=7,maxRadius=14)
    centers=np.empty((0,2)) if circles is None else circles[0,:,:2]
    supported=[]
    for r in range(4):
        for col in range(8):
            expected=np.array([16+32*col,16+32*r])
            supported.append(bool(len(centers) and np.min(np.linalg.norm(centers-expected,axis=1))<9))
    supported=np.array(supported).reshape(4,8);per_brick=[int(supported[:,2*i:2*i+2].sum()) for i in range(4)]
    if int(supported.sum())<config['minimum_studs'] or min(per_brick)<config['minimum_studs_per_brick']:
        raise Uncertain(f'Four-brick stud evidence insufficient: {per_brick} / 8 per brick')
    if partial_boundary and (int(supported.sum())<24 or min(per_brick)<5):
        raise Uncertain('Partial seam requires at least 24 studs, including five on every brick')
    center=corners.mean(axis=0)
    feature=[float(center[0]),float(center[1]),angle]
    result=dict(feature=feature,corners_px=corners.tolist(),stud_count=int(supported.sum()),studs_per_brick=per_brick,
        boundary_coverage=normalized_coverage,rectified_size=[256,128],status='observed_four_brick_row')
    if partial_boundary:result['evidence_method']='partial seam, opposite full boundary and stronger four-brick stud grid'
    overlay=image.copy();cv2.polylines(overlay,[np.round(corners).astype(np.int32)],True,(0,255,0),2)
    cv2.circle(overlay,tuple(np.round(center).astype(int)),5,(255,0,0),-1)
    cv2.putText(overlay,f"studs {per_brick}; angle {math.degrees(angle):.2f} deg",(30,700),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,0),2)
    return result,overlay,rectified


def track_row(previous,image,config):
    before,known=previous;corners=np.array(known['corners_px'],np.float32)
    reference=np.array([[0,0],[256,0],[256,128],[0,128]],np.float32)
    inverse=cv2.getPerspectiveTransform(reference,corners)
    gray=cv2.cvtColor(before,cv2.COLOR_BGR2GRAY);current=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    points=[];owners=[]
    for brick in range(4):
        polygon=cv2.perspectiveTransform(np.array([[[brick*64+5,5],[(brick+1)*64-5,5],[(brick+1)*64-5,123],[brick*64+5,123]]],np.float32),inverse)[0]
        x,y,w,h=cv2.boundingRect(polygon);x0,y0=max(0,x),max(0,y);x1,y1=min(gray.shape[1],x+w),min(gray.shape[0],y+h)
        if x1-x0<8 or y1-y0<8:raise Uncertain('Target brick feature region is clipped')
        offset=np.array([x0,y0],np.float32);mask=np.zeros((y1-y0,x1-x0),np.uint8)
        cv2.fillConvexPoly(mask,np.round(polygon-offset).astype(np.int32),255)
        found=cv2.goodFeaturesToTrack(gray[y0:y1,x0:x1],40,.015,7,mask=mask,blockSize=7)
        if found is None or len(found)<10:raise Uncertain('Insufficient per-brick tracking evidence')
        points.extend((found.reshape(-1,2)+offset).tolist());owners.extend([brick]*len(found))
    points=np.array(points,np.float32).reshape(-1,1,2);owners=np.array(owners)
    tracked,status,_=cv2.calcOpticalFlowPyrLK(gray,current,points,None,winSize=(25,25),maxLevel=3)
    if tracked is None:raise Uncertain('No row tracks')
    reverse,back_status,_=cv2.calcOpticalFlowPyrLK(current,gray,tracked,None,winSize=(25,25),maxLevel=3)
    if reverse is None:raise Uncertain('No reverse row tracks')
    good=(status.ravel()>0)&(back_status.ravel()>0)&(np.linalg.norm(reverse-points,axis=2).ravel()<1)
    if good.sum()<40:raise Uncertain('Too few consistent row tracks')
    homography,inlier=cv2.findHomography(points[good],tracked[good],cv2.RANSAC,1.5)
    if homography is None or inlier is None:raise Uncertain('Row tracking homography failed')
    valid=inlier.ravel()>0;counts=[int(((owners[good]==b)&valid).sum()) for b in range(4)]
    if min(counts)<8:raise Uncertain('Tracking does not cover every brick')
    updated=cv2.perspectiveTransform(corners[None],homography)[0]
    if np.max(np.linalg.norm(updated-corners,axis=1))>70:raise Uncertain('Row tracks exceed local-motion bounds')
    ratio=cv2.contourArea(updated)/cv2.contourArea(corners)
    if not .85<ratio<1.15:raise Uncertain('Unexpected row scale change during tracking')
    slope_top=(updated[1,1]-updated[0,1])/(updated[1,0]-updated[0,0])
    slope_bottom=(updated[2,1]-updated[3,1])/(updated[2,0]-updated[3,0])
    span=float(np.ptp(tracked[good][valid,:,0]))
    result,overlay,rectified=validate_row(image,updated,config,[span,span],math.atan((slope_top+slope_bottom)/2))
    result.update(evidence_method='tracked row with fresh per-brick stud verification',tracking_inliers_per_brick=counts)
    return result,overlay,rectified


def observe(camera,config,directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);results=[]
    camera.first_frame=True  # Flush queued frames once after motion, then use consecutive fresh frames.
    history=camera.previous_observation
    for i in range(camera.config['observation_frames']):
        image=camera.frame();cv2.imwrite(str(directory/f'frame_{i}.jpg'),image)
        try:result,overlay,rectified=detect(image,config)
        except Uncertain:
            if history is None:raise
            result,overlay,rectified=track_row(history,image,config)
        hsv=cv2.cvtColor(rectified,cv2.COLOR_BGR2HSV)
        histograms=[]
        for brick in range(4):
            patch=hsv[5:123,brick*64+5:(brick+1)*64-5]
            histogram=cv2.calcHist([patch],[0,1],None,[18,8],[0,180,0,256]).reshape(-1)
            histogram/=max(1,float(histogram.sum()));histograms.append(histogram.tolist())
        result['appearance_hists']=histograms
        history=(image,result.copy())
        cv2.imwrite(str(directory/f'overlay_{i}.jpg'),overlay);cv2.imwrite(str(directory/f'rectified_{i}.jpg'),rectified)
        results.append(result)
    features=np.array([r['feature'] for r in results]);spread=np.ptp(features,axis=0)
    if max(spread[:2])>config['maximum_frame_spread_px'] or math.degrees(spread[2])>config['maximum_frame_spread_deg']:
        raise Uncertain('Row detection is not stable across fresh frames')
    result=results[-1].copy();result.update(feature=np.median(features,axis=0).tolist(),frame_spread=spread.tolist(),
        observed_monotonic=time.monotonic(),frames=len(results))
    camera.previous_observation=history
    return result


def retention_evidence(before,after):
    """Check each visible target brick stays camera-fixed during a measured lift.

    The near-field target occupies the lower image at taught grasp height.
    Four separated regions avoid the pads and inter-brick seams. Image-space
    stability is meaningful only with the separately verified robot Z lift.
    """
    first=cv2.cvtColor(before,cv2.COLOR_BGR2GRAY);second=cv2.cvtColor(after,cv2.COLOR_BGR2GRAY)
    height,width=first.shape;results=[]
    for brick,(left,right) in enumerate(((.08,.25),(.32,.46),(.56,.70),(.80,.94))):
        mask=np.zeros_like(first);mask[round(height*.67):round(height*.93),round(width*left):round(width*right)]=255
        points=cv2.goodFeaturesToTrack(first,100,.015,7,mask=mask,blockSize=7)
        if points is None or len(points)<10:raise Uncertain(f'Brick {brick}: insufficient features for retention check')
        tracked,status,_=cv2.calcOpticalFlowPyrLK(first,second,points,None,winSize=(25,25),maxLevel=3)
        if tracked is None:raise Uncertain(f'Brick {brick}: no retention tracks')
        reverse,back_status,_=cv2.calcOpticalFlowPyrLK(second,first,tracked,None,winSize=(25,25),maxLevel=3)
        if reverse is None:raise Uncertain(f'Brick {brick}: no reverse tracks')
        good=(status.ravel()>0)&(back_status.ravel()>0)&(np.linalg.norm(reverse-points,axis=2).ravel()<1)
        if good.sum()<10:raise Uncertain(f'Brick {brick}: too few consistent tracks')
        flow=np.linalg.norm(tracked[good]-points[good],axis=2).ravel()
        median=float(np.median(flow));percentile=float(np.percentile(flow,90))
        result=dict(brick=brick,tracks=int(good.sum()),median_flow_px=median,p90_flow_px=percentile,
            retained=median<3.5 and percentile<7)
        results.append(result)
    return dict(all_four_retained=all(r['retained'] for r in results),bricks=results)


def clearance_retention_evidence(before,after):
    """Full-lift proof: every brick preserves scale while the table recedes.

    A 45 mm camera lift over an unpicked row changes its apparent scale greatly.
    Permit small settling translation only with same-scale tracks on every brick
    and a newly visible white gap above the near-field held row.
    """
    evidence=retention_evidence(before,after)
    evidence['small_lift_criterion_passed']=evidence['all_four_retained']
    gray=cv2.cvtColor(before,cv2.COLOR_BGR2GRAY);current=cv2.cvtColor(after,cv2.COLOR_BGR2GRAY)
    height,width=gray.shape
    for brick,(left,right) in zip(evidence['bricks'],((.08,.25),(.32,.46),(.56,.70),(.80,.94))):
        mask=np.zeros_like(gray);mask[round(height*.67):round(height*.93),round(width*left):round(width*right)]=255
        points=cv2.goodFeaturesToTrack(gray,100,.015,7,mask=mask,blockSize=7)
        tracked,status,_=cv2.calcOpticalFlowPyrLK(gray,current,points,None,winSize=(25,25),maxLevel=3)
        reverse,back,_=cv2.calcOpticalFlowPyrLK(current,gray,tracked,None,winSize=(25,25),maxLevel=3)
        good=(status.ravel()>0)&(back.ravel()>0)&(np.linalg.norm(reverse-points,axis=2).ravel()<1)
        fit,inliers=cv2.estimateAffinePartial2D(points[good],tracked[good],method=cv2.RANSAC,ransacReprojThreshold=1.5)
        if fit is None or inliers is None:raise Uncertain('No consistent full-lift brick geometry')
        scale=float(np.linalg.norm(fit[:,0]));count=int(inliers.sum())
        brick.update(scale_ratio=scale,geometry_inliers=count,
            retained=brick['p90_flow_px']<7 and .97<scale<1.03 and count>=10)
    hsv=cv2.cvtColor(after,cv2.COLOR_BGR2HSV)
    white=(hsv[:,:,1]<70)&(hsv[:,:,2]>150)
    rows=white[round(height*100/720):round(height*340/720),round(width*250/1280):round(width*1030/1280)].mean(axis=1)>.85
    longest=run=0
    for value in rows:
        run=run+1 if value else 0;longest=max(longest,run)
    evidence.update(white_gap_rows=longest,minimum_white_gap_rows=round(height*60/720),
        evidence_method='four scale-preserving near-field brick tracks plus exposed white separation after full lift')
    evidence['all_four_retained']=all(b['retained'] for b in evidence['bricks']) and longest>=round(height*60/720)
    return evidence


def conveyor_clear_evidence(image,roi):
    """Bounded occupancy check of the known fixed delivery footprint."""
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    if float(gray.std())<12:raise Uncertain('Conveyor view lacks enough image variation')
    x0,y0,x1,y1=roi;hsv=cv2.cvtColor(image[y0:y1,x0:x1],cv2.COLOR_BGR2HSV)
    mask=((hsv[:,:,1]>60)&(hsv[:,:,2]>55)&((hsv[:,:,0]<32)|(hsv[:,:,0]>170))).astype(np.uint8)
    count,_,stats,_=cv2.connectedComponentsWithStats(mask)
    largest=int(stats[1:,cv2.CC_STAT_AREA].max()) if count>1 else 0
    fraction=float(mask.mean())
    return dict(clear=largest<2500 and fraction<.03,colored_fraction=fraction,largest_colored_region_px=largest,roi_px=roi)
