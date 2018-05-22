import dlib
import cv2
from __future__ import print_function, division

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
