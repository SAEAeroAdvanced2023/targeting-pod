# frame size
# area size
# color range
# increment constant

import cv2
import numpy as np
import time
from gpiozero import Device, Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
from threading import Thread

# Servo init
Device.pin_factory = PiGPIOFactory()
vservo = Servo(24)
hservo = Servo(23)
vservo.value = 0
hservo.value = 0
inc = 0.0002

# Open video object
vid = cv2.VideoCapture(0)

# Processing size
res_height = 240  # 480
res_width = 426  # 852

# Set up the params
params = cv2.SimpleBlobDetector_Params()
# Change thresholds
params.minThreshold = 10
params.maxThreshold = 200
# Filter by Area.
params.filterByArea = True
params.minArea = 3000
params.maxArea = 10000000000
# Filter by Circularity
params.filterByCircularity = False
params.minCircularity = 0.5
# Filter by Convexity
params.filterByConvexity = False
params.minConvexity = 0.50
# Filter by Inertia
params.filterByInertia = False
params.minInertiaRatio = 0.01
# Filter by color
params.filterByColor = True
params.blobColor = 255

# Setup detector
detector = cv2.SimpleBlobDetector_create(params)

# Mode
# "ready": no math, user controllable
# "manual": math, user controllable
# "auto": no math, not user controllable
mode = "auto"


def normalize(x):
    if x > 1: x = 1
    if x < -1: x = -1
    return x



def calculate_point():
    print("Math!")

input_speed = 0.01

def manual():
    calculate_point()

def auto():
    if len(keypoints) > 0:
        #calculate_point()
        hdif = abs(res_width / 2 - keypoints[0].pt[0])
        if keypoints[0].pt[0] > res_width / 2:
            hservo.value = normalize(hservo.value - (hdif * inc))
        else:
            hservo.value = normalize(hservo.value + (hdif * inc))
        vdif = abs(res_height / 2 - keypoints[0].pt[1])
        if keypoints[0].pt[1] > res_height / 2:
            vservo.value = normalize(vservo.value - (vdif * inc))
        else:
            vservo.value = normalize(vservo.value + (vdif * inc))


# Loop through video
while True:
    #stime = time.time()
    ret, frame = vid.read()
    frame = cv2.resize(frame, (res_width, res_height))

    # Convert to hsv color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define mask for color
    # Red
    lower_bound = np.array([0, 2, 100])  # [0, 100, 100]
    upper_bound = np.array([5, 255, 255])  # [5, 255, 255]
    mask1 = cv2.inRange(hsv, lower_bound, upper_bound)
    lower_bound = np.array([160, 2, 100])  # [160, 100, 100]
    upper_bound = np.array([180, 255, 255])  # [180, 255, 255]
    mask2 = cv2.inRange(hsv, lower_bound, upper_bound)
    mask = mask1 + mask2
    # Green
    #     lower_bound = np.array([40, 100, 25])
    #     upper_bound = np.array([90, 255, 255])
    #     mask = cv2.inRange(hsv, lower_bound, upper_bound)
    # Blue
    #     lower_bound = np.array([106, 100, 100])
    #     upper_bound = np.array([125, 255, 255])
    #     mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Find the colors within the boundaries
    keypoints = detector.detect(mask)
    # draw circles over keypoints
    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]),
                                          (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    for kp in keypoints:
        cv2.circle(im_with_keypoints, (int(kp.pt[0]), int(kp.pt[1])), 2, (0, 255, 0), -1)
    # print(str(kp.pt[0]) + "," + str(kp.pt[1]))

    cv2.circle(im_with_keypoints, (int(res_width / 2), int(res_height / 2)), 2, (255, 255, 0), -1)
    if len(keypoints) > 0:
        cv2.line(im_with_keypoints, (int(keypoints[0].pt[0]), int(keypoints[0].pt[1])),
                 (int(res_width / 2), int(res_height / 2)), (0, 255, 0), 1)

    # Show results
    cv2.imshow('Mask Frame', mask)
    cv2.imshow('Detection Frame', im_with_keypoints)

    auto()

    if cv2.waitKey(25) & 0xFF == ord("m"):
        mode = "manual"
    elif cv2.waitKey(25) & 0xFF == ord('a'):
        mode = "auto"
    elif cv2.waitKey(25) & 0xFF == ord('r'):
        mode = "ready"
    elif cv2.waitKey(25) & 0xFF == ord('q'):
        exit(0)

    #etime = time.time()
    #print(str(etime - stime))

# Reset servos
vservo.value = 0
hservo.value = 0
# Release the video capture object
vid.release()
# Closes all the windows currently opened.
cv2.destroyAllWindows()
