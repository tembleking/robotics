from typing import List, Tuple

import cv2
import numpy as np

from robotics.geometry import Point


class Camera:

    def __init__(self,
                 video_capturer: cv2.VideoCapture, robot_to_find: str):
        self.video_capturer = video_capturer
        self._robot_to_find = robot_to_find
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 100
        params.maxArea = 60000
        self.blob_detector = cv2.SimpleBlobDetector_create(params)
        self.mask_lower_min_hsv = np.array([0, 100, 0])
        self.mask_lower_max_hsv = np.array([8, 255, 255])
        self.mask_higher_min_hsv = np.array([160, 100, 0])
        self.mask_higher_max_hsv = np.array([180, 255, 255])

    def _get_frame(self) -> np.ndarray:
        ok, frame = self.video_capturer.read()
        if ok:
            return frame
        return None

    def _get_mask(self, frame) -> np.ndarray:
        # Retrieves a mask for an image that filters the HSV red from hue: [170-180] and [0-10]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_lower = cv2.inRange(hsv, self.mask_lower_min_hsv, self.mask_lower_max_hsv)
        mask_higher = cv2.inRange(hsv, self.mask_higher_min_hsv, self.mask_higher_max_hsv)
        mask = cv2.bitwise_or(mask_lower, mask_higher)
        return mask

    def _detect_blobs(self, frame) -> List[cv2.KeyPoint]:
        mask = self._get_mask(frame)
        detections = self.blob_detector.detect(255 - mask)
        return detections

    def get_frame(self) -> np.ndarray:
        frame = self._get_frame()
        frame = self._get_frame() if frame is None else frame
        frame = self._get_frame() if frame is None else frame
        return frame

    def get_blob_position_and_size(self) -> Tuple[Point, float]:
        frame = self.get_frame()
        keypoints = self._detect_blobs(frame)
        if len(keypoints) == 0:
            return None, None

        best_keypoint = keypoints[0]
        for keypoint in keypoints:
            if keypoint.size > best_keypoint.size:
                best_keypoint = keypoint

        return Point(best_keypoint.pt[0], best_keypoint.pt[1]), best_keypoint.size

    def is_ball_within_claws(self) -> bool:
        frame = self.get_frame()
        mask = self._get_mask(frame)[380:, :360] / 255
        mean = mask.mean()
        print('Mean of ball within claws: %s' % mean)
        return mean > 0.25

    def get_homography_robot_position(self) -> str:
        pass
