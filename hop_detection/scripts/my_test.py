#!/usr/bin/env python
from __future__ import print_function, division
#from .multi_detector import ObjectDetector
import numpy as np
import cv2
#from tracking import *
import dlib
import argparse


ap = argparse.ArgumentParser()
#ap.add_argument("-d", "--detector", required=True,
#               help="path to trained detector to load...")
#ap.add_argument("-t", "--tracker", required=True, help="tracker to use...")
ap.add_argument("-c", "--camera", required=True, help="camera id")
args = vars(ap.parse_args())




class ObjectDetector(object):
    def __init__(self,options=None,loadPath=None):
        #create detector options
        self.options = options
        if self.options is None:
            self.options = dlib.simple_object_detector_training_options()

        self._detectors = [] #= dlib.fhog_object_detector(loadPath)

    def add_detector(self, det):
        self._detectors.append(dlib.fhog_object_detector(det))

    def _prepare_annotations(self,annotations):
        annots = []
        for (x,y,xb,yb) in annotations:
            annots.append([dlib.rectangle(left=int(x),top=int(y),right=int(xb),bottom=int(yb))])
        return annots

    def _prepare_images(self,imagePaths):
        images = []
        for imPath in imagePaths:
            image = cv2.imread(imPath)
            image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
            images.append(image)
        return images

    def fit(self, imagePaths, annotations, visualize=False, savePath=None):
        annotations = self._prepare_annotations(annotations)
        images = self._prepare_images(imagePaths)
        self._detector = dlib.train_simple_object_detector(images, annotations, self.options)

        #visualize HOG
        if visualize:
            win = dlib.image_window()
            win.set_image(self._detector)
            dlib.hit_enter_to_continue()

        #save detector to disk
        if savePath is not None:
            self._detector.save(savePath)

        return self

    def predict(self,image):
        preds = dlib.fhog_object_detector.run_multiple(self._detectors, image, 2)
        if len(preds[2]) > 0:
            boxes = []
            idx = preds[2][0]
            for box in preds[0]:
                (x,y,xb,yb) = [box.left(),box.top(),box.right(),box.bottom()]
                boxes.append((x,y,xb,yb))
            print(boxes)
            print(idx)
            return boxes

        # preds_list = []
        # for d in self._detectors:
        #     boxes = d(image)
        #     preds = []
        #     for box in boxes:
        #         (x,y,xb,yb) = [box.left(),box.top(),box.right(),box.bottom()]
        #         preds.append((x,y,xb,yb))
        #     preds_list.append(preds)
        #print(preds)
        return None

    def detect(self,image,annotate=None, wait=0):
        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        preds = self.predict(image)
        for (x,y,xb,yb) in preds:
            image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)

            #draw and annotate on image
            cv2.rectangle(image,(x,y),(xb,yb),(0,0,255),2)
            if annotate is not None and type(annotate)==str:
                cv2.putText(image,annotate,(x+5,y-5),cv2.FONT_HERSHEY_SIMPLEX,1.0,(128,255,0),2)
        cv2.imshow("Detected",image)
        cv2.waitKey(wait)
        if len(preds)==0:
            return 0
        else:
            return 1

test_detector = ObjectDetector()
test_detector.add_detector("/home/nvidia/catkin_ws/src/hopai/hop_detection/svms/enemy-front-detector.svm")
test_detector.add_detector("/home/nvidia/catkin_ws/src/hopai/hop_detection/svms/enemy-back-detector.svm")
test_detector.add_detector("/home/nvidia/catkin_ws/src/hopai/hop_detection/svms/enemy-left-detector.svm")
test_detector.add_detector("/home/nvidia/catkin_ws/src/hopai/hop_detection/svms/enemy-right-detector.svm")
test_detector.add_detector("/home/nvidia/catkin_ws/src/hopai/hop_detection/svms/enemy_detector.svm")


annotate = 'enemy'
class OpenCVTracker(object):

    def __init__(self, roiPts, tracker_name='KCF', tracker_id=2):
        (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
        # set initial points
        tracker_types = ['BOOSTING', 'MIL',
                         'KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
        tracker_type = tracker_name
        if int(minor_ver) < 3:
            self.tracker = cv2.Tracker_create(tracker_type)
        else:
            if tracker_type == 'BOOSTING':
                self.tracker = cv2.TrackerBoosting_create()
            if tracker_type == 'MIL':
                self.tracker = cv2.TrackerMIL_create()
            if tracker_type == 'KCF':
                self.tracker = cv2.TrackerKCF_create()
            if tracker_type == 'TLD':
                self.tracker = cv2.TrackerTLD_create()
            if tracker_type == 'MEDIANFLOW':
                self.tracker = cv2.TrackerMedianFlow_create()
            if tracker_type == 'GOTURN':
                self.tracker = cv2.TrackerGOTURN_create()

        self.roiPts = np.array(roiPts)
        self.roiBox = None

    def orderPoints(self):
        assert len(self.roiPts) == 2
        return self.roiPts[np.argsort(self.roiPts.sum(axis=1))]

    def track(self, image):
        self.image = image
        # order the points and gather top-left and bottom-right points
        pts = self.orderPoints().tolist()
        tl, br = pts

        # if tracking is not yet started initialize it by setting the roiBox and hist
        if self.roiBox is None:
            self.roiBox = (tl[0], tl[1], br[0] - tl[0], br[1] - tl[1])
            if not self.tracker.init(self.image, self.roiBox):
                return None
        try:
            ok, self.roiBox = self.tracker.update(self.image)
            if ok:
                return self.roiBox

        except Exception:
            return None

class CamRectify:
    def __init__(self,
                 mtx=np.array([[476.185933, 0, 318.35332], [
                              0, 446.410525, 219.271978], [0, 0, 1]]),
                 dist=np.array([-0.381355, 0.128511, -0.004245, 0.001909, 0])):
        self.mtx = mtx
        self.dist = dist
        self.start = False

    def rectify(self, img):
        # if not self.start:
        h, w = img.shape[:2]
        # self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(
        #     self.mtx, self.dist, (w, h), 1, (w, h))
        # print(self.newcameramtx, self.roi)
        self.start = True
        img = cv2.undistort(img, self.mtx, self.dist)
            # ,
            #                 newCameraMatrix=self.newcameramtx)
        # crop the image
        # x, y, w, h = self.roi
        # print(self.roi)
        # img = img[y:y + h, x:x + w]
        return img


def centered_crop(img):
    width = np.size(img, 1)
    height = np.size(img, 0)
    new_width = min([width, height])
    new_height = new_width
    left = int((width - new_width) / 2.)
    top = int((height - new_height) / 2.)
    right = int((width + new_width) / 2.)
    bottom = int((height + new_height) / 2.)
    cImg = img[top:bottom, left:right]
    return cImg


def callback(frame, t):
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)    # makes the blues image look real colored
    frame = centered_crop(frame)
    frame = cv2.resize(frame, (256, 256))
    preds = test_detector.predict(frame)
    print(preds)
    if preds is not None:
        for (x, y, xb, yb) in preds:
            #image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
            # draw and annotate on image
            cv2.rectangle(frame, (x, y), (xb, yb), (0, 0, 255), 2)
            if annotate is not None and type(annotate) == str:
                cv2.putText(frame, annotate, (x + 5, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (128, 255, 0), 2)
    return frame


tracking = 0
enemy_tracker = None


def tracker_callback(frame, tracker_name):
    global tracking
    global enemy_tracker
    # cr = CamRectify(imtx, idist)
    #frame = centered_crop(frame)
    # h, w = frame.shape[:2]
    # frame = cv2.resize(frame, (int(w / 5), int(h/5)))
    # frame = cr.rectify(frame)
    if enemy_tracker is not None:
        tracking = tracking + 1
        track_pts = enemy_tracker.track(frame)
        if track_pts is not None:
            (x, y, w, h) = track_pts
            x = int(x)
            y = int(y)
            w = int(w)
            h = int(h)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            if annotate is not None and type(annotate) == str:
                cv2.putText(frame, annotate, (x + 5, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (128, 255, 0), 2)
        else:
            tracking = 0
    # init
    if tracking == 0 or tracking % 10 == 0:
        preds = test_detector.predict(frame)
        if preds is not None:
            for (x, y, xb, yb) in preds:
                # enemy_tracker = CorrelationTracker([(x,y),(xb, yb)])
                enemy_tracker = OpenCVTracker([(x, y), (xb, yb)], tracker_name)
                cv2.rectangle(frame, (x, y), (xb, yb), (0, 0, 255), 2)
                if annotate is not None and type(annotate) == str:
                    cv2.putText(frame, annotate, (x + 5, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (128, 255, 0), 2)
            tracking = 1

    return frame


cap = cv2.VideoCapture(int(args['camera']))
imtx = np.array([[476.185933, 0, 318.35332], [
                0, 446.410525, 219.271978], [0, 0, 1]])
idist = np.array([-0.381355, 0.128511, -0.004245, 0.001909, 0])
#cr = CamRectify(imtx, idist)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    if ret:
        # Our operations on the frame come here
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        #frame = cr.rectify(frame)
        frame = centered_crop(frame)
        frame = cv2.resize(frame, (256, 256))
        frame = tracker_callback(frame, 'KCF')
        # Display the resulting frame
        #cv2.imshow('frame', frame)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
