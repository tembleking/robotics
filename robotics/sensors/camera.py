import os
import sys
from typing import List, Tuple

import cv2
import numpy as np

from robotics.geometry import Point


class Camera:

    def __init__(self,
                 video_capturer: cv2.VideoCapture, robot_to_find: str,
                 r2d2_template: np.ndarray, bb8_template: np.ndarray):
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
        self.r2d2_template = r2d2_template
        self.bb8_template = bb8_template
        self.min_match_count = 20
        self.min_match_object_found = 15
        self.count = 0
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
        try:
            frame = self._get_frame()
            frame = self._get_frame() if frame is None else frame
            frame = self._get_frame() if frame is None else frame
        except Exception:
            frame = None
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

    def _match_images(self, reference_image: np.ndarray, img2_bgr: np.ndarray):
     
        # Feature extractor uses grayscale images
        img1 = cv2.cvtColor(reference_image, cv2.COLOR_BGR2GRAY)
        img2 = cv2.cvtColor(img2_bgr, cv2.COLOR_BGR2GRAY)
        filename = "homograpy" + str(self.count) + ".png"
        self.count += 1
        cv2.imwrite(filename, img2_bgr)
        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3: # CURRENT RASPBERRY opencv version is 2.4.9
            # Initiate ORB detector --> you could use any other detector, but this is the best performing one in this version
            binary_features = True

            detector = cv2.ORB()
        else: 
            # Initiate BRISK detector --> you could use any other detector, including NON binary features (SIFT, SURF)
            # but this is the best performing one in this version
            binary_features=True
            detector = cv2.BRISK_create()
            

        # find the keypoints and corresponding descriptors
        kp1, des1 = detector.detectAndCompute(img1,None)
        kp2, des2 = detector.detectAndCompute(img2,None)

        if des1 is None or des2 is None:
            print("WARNING: empty detection?")
            return False
        if len(des1) < self.min_match_count or len(des2) < self.min_match_count:
            print("WARNING: not enough FEATURES (im1: %d, im2: %d)" %(len(des1), len(des2)) )
            return False
        print(" FEATURES extracted (im1: %d, im2: %d)" %(len(des1), len(des2)) )
            

        if binary_features:
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des1,des2)
            matches = sorted(matches, key = lambda x:x.distance)
            good = matches
        else:
            FLANN_INDEX_KDTREE = 0
            index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
            search_params = dict(checks = 50)
            flann = cv2.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch(des1,des2,k=2)
            # store all the good matches as per Lowe's ratio test.
            good = []
            for m,n in matches:
                if m.distance < 0.7*n.distance:
                    good.append(m)

        print(" Initial matches found: %d" %(len(good)))

        if len(good)>self.min_match_count:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            H_21, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)
            matchesMask = mask.ravel().tolist()
            num_robust_matches = np.sum(matchesMask)
            if num_robust_matches < self.min_match_object_found:
                found = None
                print("NOT enough ROBUST matches found - %d (required %d)" % 
                    (num_robust_matches, self.min_match_object_found))
                return found
            h,w = img1.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,H_21)
            img2_res = cv2.polylines(img2_bgr, [np.int32(dst)], True, 
                                     color=(255,255,255), thickness=3)
            cv2.imwrite("homography_matches.png", img2_res)
            found = (dst[3,0,0] - dst[0,0,0]) / 2 + dst[0,0,0]
            print("ROBUST matches found - %d (out of %d) --> OBJECT FOUND" % (np.sum(matchesMask), len(good)))
        else:
            print("Not enough initial matches are found - %d (required %d)" % (len(good), self.min_match_count))
            matchesMask = None
            found = None

        return found


    def get_homography_robot_position(self) -> str:
        img = self.get_frame()
        img = self.get_frame()
        img = self.get_frame()
        img = self.get_frame()
        img = self.get_frame()
        while img is None:
            img = self.get_frame()
        if img is None:
            return None
        target = self._robot_to_find
        if target != "r2d2" and target != "bb8":
            print("Unknown target robot")
            return None
        
        r2d2_x = self._match_images(self.r2d2_template, img)
        print(r2d2_x)
        #bb8_x = self._match_images(self.bb8_template, img)
        bb8_x = 300
        if r2d2_x is None or bb8_x is None:
            return "left" # Fallback
        if target == "r2d2":
            print("Target is r2d2")
            if r2d2_x < bb8_x:
                print("r2d2 is left")
                return "left"
            else:
                print("r2d2 is right")
                return "right"
        else:
            print("Target is bb8")
            if bb8_x < r2d2_x:
                print("bb8 is left")
                return "left"
            else:
                print("bb8 is right")
                return "right"
