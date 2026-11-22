"""Track only the four already identified bricks through approach and lift."""
import cv2
import numpy as np

from runtime import Stop


class RowTrack:
    def __init__(self,image,corners):
        self.gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        self.corners=np.asarray(corners,np.float32)
        inverse=cv2.getPerspectiveTransform(np.array([[0,0],[256,0],[256,128],[0,128]],np.float32),self.corners)
        points=[];owners=[];self.polygons=[]
        for b in range(4):
            polygon=cv2.perspectiveTransform(np.array([[[64*b+6,8],[64*(b+1)-6,8],
                [64*(b+1)-6,120],[64*b+6,120]]],np.float32),inverse)[0]
            self.polygons.append(polygon)
            x,y,w,h=cv2.boundingRect(polygon);x0,y0=max(0,x),max(0,y);x1,y1=min(self.gray.shape[1],x+w),min(self.gray.shape[0],y+h)
            if x1-x0<8 or y1-y0<8:raise Stop('Target brick feature region is clipped')
            offset=np.array([x0,y0],np.float32);mask=np.zeros((y1-y0,x1-x0),np.uint8)
            cv2.fillConvexPoly(mask,np.round(polygon-offset).astype(np.int32),255)
            found=cv2.goodFeaturesToTrack(self.gray[y0:y1,x0:x1],80,.012,6,mask=mask,blockSize=7)
            if found is None or len(found)<12:raise Stop('Insufficient per-brick image features')
            points.extend(found.reshape(-1,2)+offset);owners.extend([b]*len(found))
        self.points=np.asarray(points,np.float32).reshape(-1,1,2);self.owners=np.asarray(owners)
        self.reference=None

    def update(self,image):
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        tracked,status,_=cv2.calcOpticalFlowPyrLK(self.gray,gray,self.points,None,winSize=(31,31),maxLevel=3)
        if tracked is None:raise Stop('No four-brick tracks')
        reverse,back,_=cv2.calcOpticalFlowPyrLK(gray,self.gray,tracked,None,winSize=(31,31),maxLevel=3)
        if reverse is None:raise Stop('No reverse four-brick tracks')
        good=(status.ravel()>0)&(back.ravel()>0)&(np.linalg.norm(reverse-self.points,axis=2).ravel()<1.2)
        if min(int(((self.owners==b)&good).sum()) for b in range(4))<10:
            raise Stop('A target brick lost visual correspondence')
        # Each separate brick is the rigid unit. Stud relief and lens distortion
        # during descent do not obey one exact plane across the entire row.
        valid=np.zeros_like(good);counts=[];polygons=[]
        for b in range(4):
            indices=np.flatnonzero(good&(self.owners==b))
            local,inliers=cv2.findHomography(self.points[indices],tracked[indices],cv2.RANSAC,1.5)
            if local is None or inliers is None or int(inliers.sum())<10:
                raise Stop(f'Brick{b} lacks ten geometric image correspondences')
            valid[indices]=inliers.ravel()>0;counts.append(int(inliers.sum()))
            polygons.append(cv2.perspectiveTransform(self.polygons[b][None],local)[0])
        H,_=cv2.findHomography(self.points[valid],tracked[valid],0)
        if H is None:raise Stop('Row display geometry could not be updated')
        updated=cv2.perspectiveTransform(self.corners[None],H)[0]
        ratio=cv2.contourArea(updated)/cv2.contourArea(self.corners)
        if not .80<ratio<1.25 or np.max(np.linalg.norm(updated-self.corners,axis=1))>70:
            raise Stop('Image motion exceeds bounded local row tracking')
        self.corners=updated;self.gray=gray
        self.last_geometric_counts=counts
        self.points=tracked[valid];self.owners=self.owners[valid]
        if self.reference is not None:self.reference=self.reference[valid]
        self.polygons=polygons
        if self.reference is None:
            # Before closing, camera descent changes viewpoint and specular
            # corners disappear. Replenish inside each already tracked brick;
            # never discard all original features before identity is verified.
            # Once mark_grasp freezes the retention reference, replenishment is
            # prohibited so a failed pick cannot acquire substitute points.
            for b,polygon in enumerate(self.polygons):
                if int((self.owners==b).sum())>=40:continue
                x,y,w,h=cv2.boundingRect(polygon);x0,y0=max(0,x),max(0,y);x1,y1=min(gray.shape[1],x+w),min(gray.shape[0],y+h)
                if x1-x0<8 or y1-y0<8:raise Stop('Tracked brick region left the camera image')
                offset=np.array([x0,y0],np.float32);mask=np.zeros((y1-y0,x1-x0),np.uint8)
                cv2.fillConvexPoly(mask,np.round(polygon-offset).astype(np.int32),255)
                found=cv2.goodFeaturesToTrack(gray[y0:y1,x0:x1],80,.012,6,mask=mask,blockSize=7)
                if found is None or len(found)<12:raise Stop('Cannot refresh features on identified brick')
                keep=self.owners!=b;new=(found.reshape(-1,2)+offset).reshape(-1,1,2)
                self.points=np.concatenate([self.points[keep],new]);self.owners=np.r_[self.owners[keep],np.full(len(new),b)]

    def mark_grasp(self):self.reference=self.points.copy()

    def retention(self,measured_lift):
        if self.reference is None or measured_lift<.045:raise Stop('Full measured clearance lift is missing')
        results=[]
        for b in range(4):
            which=self.owners==b;before=self.reference[which];after=self.points[which]
            fit,inlier=cv2.estimateAffinePartial2D(before,after,method=cv2.RANSAC,ransacReprojThreshold=1.5)
            if fit is None or inlier is None:raise Stop('Retained brick geometry is uncertain')
            scale=float(np.linalg.norm(fit[:,0]));flow=np.linalg.norm(after-before,axis=2).ravel()
            count=int(inlier.sum());median=float(np.median(flow));p90=float(np.percentile(flow,90))
            results.append(dict(brick=b,inliers=count,scale=scale,median_flow_px=median,p90_flow_px=p90,
                retained=count>=10 and .97<scale<1.03 and median<3.5 and p90<7))
        result=dict(all_four_retained=all(r['retained'] for r in results),bricks=results,
                    measured_lift_m=measured_lift,method='Identified brick tracks, camera-fixed scale and position during45mm+ vertical lift')
        if not result['all_four_retained']:raise Stop('Full-lift verification failed: '+str(result))
        return result
