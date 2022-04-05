import cv2
import numpy as np
from typing import List
from robotics.geometry import Point

class Camera():

    def __init__(self, min_light_settings: np.ndarray,  min_dark_settings: np.ndarray, params: cv2.SimpleBlobDetector_Params):
        self.video_capturer = cv2.VideoCapture(0)
        self.blob_detector = cv2.SimpleBlobDetector_create(params)
        self.min_light_settings = min_light_settings
        self.min_dark_settings = min_dark_settings

    def get_frame(self) -> np.ndarray:
        ok, frame = self.video_capturer.read()
        if ok:
            return frame

    def _detect_blobs(self, frame) -> List[cv2.Feature2D]:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        higher_hsv = np.array([8, 255, 255])
        mask = cv2.inRange(hsv, self.min_light_settings, higher_hsv)
        return self.blob_detector.detect(255 - mask)

    def get_blob_position_and_size(self):
        frame = self.get_frame()
        keypoints = self._detect_blobs(frame)
        best_keypoint = cv2.Feature2D()
        for keypoint in keypoints:
            if keypoint.size > best_keypoint.size:
                best_keypoint = keypoint

        return Point(best_keypoint.pt[0], best_keypoint[1].y), best_keypoint.size
