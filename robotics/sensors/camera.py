import cv2
import numpy as np
from typing import List, Tuple
from robotics.geometry import Point


class Camera:

    def __init__(self,
                 video_capturer: cv2.VideoCapture):
        self.video_capturer = video_capturer
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 200
        params.maxArea = 60_000
        self.blob_detector = cv2.SimpleBlobDetector_create(params)
        self.mask_lower_min_hsv = np.array([0, 125, 0])
        self.mask_lower_max_hsv = np.array([10, 255, 255])
        self.mask_higher_min_hsv = np.array([170, 125, 0])
        self.mask_higher_max_hsv = np.array([180, 255, 255])

    def _get_frame(self) -> np.ndarray:
        ok, frame = self.video_capturer.read()
        if ok:
            return frame

    def _get_mask(self, frame) -> np.ndarray:
        # Retrieves a mask for a image that filters the HSV red from hue: [170-180] and [0-10]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_lower = cv2.inRange(hsv, self.mask_lower_min_hsv, self.mask_lower_max_hsv)
        mask_higher = cv2.inRange(hsv, self.mask_higher_min_hsv, self.mask_higher_max_hsv)
        mask = cv2.bitwise_or(mask_lower, mask_higher)
        return mask

    def _detect_blobs(self, frame) -> List[cv2.KeyPoint]:
        mask = self._get_mask(frame)
        detections = self.blob_detector.detect(255 - mask)
        return detections

    def get_blob_position_and_size(self) -> Tuple[Point, float]:
        frame = self._get_frame()
        keypoints = self._detect_blobs(frame)
        if len(keypoints) == 0:
            return None, None

        best_keypoint = keypoints[0]
        for keypoint in keypoints:
            if keypoint.size > best_keypoint.size:
                best_keypoint = keypoint

        return Point(best_keypoint.pt[0], best_keypoint.pt[1]), best_keypoint.size
