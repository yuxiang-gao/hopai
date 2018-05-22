import numpy as np
import cv2


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
