import cv2
import numpy as np

# TODO: Make a class
 
d_goal = 330    # X coordinate destiny relative to the camera
a_goal = 170    # Area destiny

# Open the camera
# TODO: Put this in a init function with the blob detector
cap = cv2.VideoCapture(0) 

# Create Blob Detector
params = cv2.SimpleBlobDetector_Params()

# Thresholds (no clue about the meaning of this values)
params.minThreshold = 400
params.maxThreshold = 4000

# Filter by Area 
#   hard to define, but this works quite well
#   even though we might have to lower the minArea
params.filterByArea = True
params.minArea = 1000
params.maxArea = 50000

# Filter by Circularity
# The area has to define a circle
params.filterByCircularity = True
params.minCircularity = 0.01

params.filterByConvexity = True
params.minConvexity = 0.4
params.maxConvexity = 0.9

# Dont filter by color (we do it with hsv values), Convexity or Inertia
params.filterByColor = False

params.filterByInertia = False
params.minInertiaRatio = 0.1
params.maxInertiaRatio = 1

# Create a detector with the parameters depending on version of cv2
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
	detector = cv2.SimpleBlobDetector(params)
else :
	detector = cv2.SimpleBlobDetector_create(params)

# HSV min values depending of the room ilumination
minDarkSettings = [0,85,0]
minLightSettings = [0,124,0]

# Last known angular speed
lastKnownW = 0

no_ofLostFrames = 0
maxLostFrames = 120 # 2 seconds of images (if 60fps)

targetReached = False
lostBall = False

dampingFactorSize = 1 / 1;
dampingFactorDistance = 1 / 1;

sizeThreshold = a_goal - 10
distanceThreshold = d_goal - 20   # Check must be done using absolute values

# When doing this with the robot, delete while(true) and change it with a
#   function call each x ms
while(True):
    # Get the frame of the camera
    ret, frame = cap.read()
    
    # We couldn't get a frame of the camera
    if not ret:
        no_ofLostFrames+=1
        if no_ofLostFrames > maxLostFrames:
            print("Critical error while retreaving camera info")
            break
        continue
    
    no_ofLostFrames = 0
    
    # Convert color to hsv because it is easy to track colors in this color model
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_hsv = np.array(minLightSettings)
    higher_hsv = np.array([8, 255, 255])

    # Apply the cv2.inrange method to create a mask
    mask = cv2.inRange(hsv, lower_hsv, higher_hsv)
    
    # Apply the mask on the image to extract the original color
    frame = cv2.bitwise_and(frame, frame, mask=mask)

    keypoints = detector.detect(255-mask)
    # pt[0] = X         >  Angular <
    # pt[1] = Y         >  Unused  < 
    # pt.size = size    >  Lineal  <
    
    # Is there a blob in screen?
    if len(keypoints) > 0:
        # Algorithm that chooses best blob depending on its size
        # The bigger the better
        bestPoint = keypoints[0]
        bestSize = keypoints[0].size
        for kp in keypoints:
            if bestSize > kp.size:
                bestPoint = kp             
        
        # Check that we didnt lost the ball
        if targetReached:
            # When reached, ball size will not be the desired one 
            lostBall = (bestSize < sizeThreshold - 20) or (abs(bestPoint.pt[0]) < distanceThreshold)
        
        # We didn't reached the goal or lost the ball
        if (not targetReached) or (lostBall):        
            w = dampingFactorDistance * (d_goal - bestPoint.pt[0])      # Values (-250,250) without damping
            v = dampingFactorSize * (a_goal - bestPoint.size)           # values (0,170) without damping
            print("Angular speed:",w,"    \t Linear Speed:",v)
            lastKnownW = w

        # We have reached the objective
        if (bestSize > sizeThreshold) and (abs(bestPoint.pt[0]) < distanceThreshold):
            # Change prints for usuful code
            print("Finishing ball detection alorithm")
            targetReached = True
            lostBall = False 
            print("Starting ball mantaining algorithm")
            
            print("Adjustating position and closing claws")
        
    else:
        targetReached = False
        print("Ball is nowhere to be seen")
        if lastKnownW < 0:
            print("\tTurning Right")
            # Rotate in place with angular speed > 0
            
        else:
            print("\tTurning Left")
            # Rotate in place with angular speed < 0

    
    
    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]),
	(255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    cv2.imshow('keypoints', im_with_keypoints)

    # Press q to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()