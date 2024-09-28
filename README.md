# Hand-Control-Using-OpenCV-and-Python

## Overview

**Hand-Control-Using-OpenCV-and-Python** is a project that allows you to control your computer's cursor using hand gestures. The system leverages computer vision techniques using OpenCV and the cvzone library's HandTrackingModule to detect and interpret hand movements in real-time. 

Key Features:
- Move the cursor by moving your index finger.
- Right-click by touching your thumb and index finger together.
- Left-click by tapping your index finger.
- Scroll by extending both the index and middle fingers.

## Finger patterns

![alt text](https://www.mdpi.com/engproc/engproc-58-00069/article_deploy/html/images/engproc-58-00069-g001.png)

## Installation

To run this project, you will need Python and the following libraries installed on your machine:

1. OpenCV
2. cvzone
3. numpy
4. mouse
5. keyboard

You can install the required libraries using pip:

```sh
pip install opencv-python cvzone numpy mouse keyboard
```

## Usage

1. Clone the repository:

```sh
git clone https://github.com/yourusername/Hand-Control-Using-OpenCV-and-Python.git
cd Hand-Control-Using-OpenCV-and-Python
```

2. Run the script:

```sh
python hand_control.py
```

## Code Explanation

### Main Script

The main script, `hand_control.py`, initializes the hand detector, captures video from the webcam, and processes hand movements to control the cursor.

```python
import cv2
import numpy as np
import mouse
import keyboard
from cvzone.HandTrackingModule import HandDetector
import time
import threading

# Initialize hand detector and other variables
detector = HandDetector(detectionCon=0.8, maxHands=1)
cam_w, cam_h = 640, 480  # Camera resolution
screen_w, screen_h = 1920, 1080  # Screen resolution
positions = []  # Store positions for smoothing
smoothing_window = 5  # Number of positions to average
frameR = 100  # Frame reduction to avoid edge issues
l_delay = 0
click_thread_active = False
prev_dist = None
scroll_smoothness = 5
scroll_history = []
prev_hand_x = None
swipe_threshold = 200  # Adjust based on your need

def l_clk_delay():
    global l_delay
    global click_thread_active
    time.sleep(1)
    l_delay = 0
    click_thread_active = False

# Function to compute the average of the last few positions
def average_position(positions):
    x_avg = sum(p[0] for p in positions) // len(positions)
    y_avg = sum(p[1] for p in positions) // len(positions)
    return x_avg, y_avg

# Capture video from camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_w)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_h)

while True:
    success, img = cap.read()
    if not success:
        print("Error: Failed to capture image")
        break

    img = cv2.flip(img, 1)
    hands, img = detector.findHands(img, flipType=False)
    cv2.rectangle(img, (frameR, frameR), (cam_w - frameR, cam_h - frameR), (255, 0, 255), 2)

    if hands:
        hand = hands[0]
        lmlist = hand['lmList']
        fingers = detector.fingersUp(hand)

        ind_x, ind_y = lmlist[8][0], lmlist[8][1]
        pip_x, pip_y = lmlist[7][0], lmlist[7][1]
        mid_x, mid_y = lmlist[12][0], lmlist[12][1]
        thumb_x, thumb_y = lmlist[4][0], lmlist[4][1]
        base_x, base_y = lmlist[5][0], lmlist[5][1]
        hand_center_x = hand['center'][0]

        # Append the current position to the positions list
        positions.append((ind_x, ind_y))
        if len(positions) > smoothing_window:
            positions.pop(0)

        # Calculate the average position
        avg_x, avg_y = average_position(positions)

        # Check if the index finger is straight
        if fingers[1] == 1 and abs(avg_y - base_y) > 30 and abs(avg_y - mid_y) >= 20:
            move_x = int(np.interp(avg_x, (frameR, cam_w - frameR), (0, screen_w)))
            move_y = int(np.interp(avg_y, (frameR, cam_h - frameR), (0, screen_h)))
            mouse.move(move_x, move_y)

        # Right click when thumb touches index finger
        if abs(thumb_x - avg_x) < 20 and abs(thumb_y - avg_y) < 20:
            if l_delay == 0 and not click_thread_active:
                mouse.click(button="right")
                l_delay = 1
                click_thread_active = True
                threading.Thread(target=l_clk_delay).start()
            continue

        # Tap detection
        if fingers[1] == 1 and fingers[2] == 0 and abs(avg_y - mid_y) >= 20:
            if abs(avg_y - pip_y) < 20:
                if l_delay == 0 and not click_thread_active:
                    mouse.click(button="left")
                    l_delay = 1
                    click_thread_active = True
                    threading.Thread(target=l_clk_delay).start()

        # Scroll detection
        if fingers[1] == 1 and fingers[2] == 1:
            if abs(avg_y - mid_y) < 20:
                mouse.wheel(delta=-1)
                time.sleep(0.1)
            elif abs(avg_y - mid_y) >= 20:
                mouse.wheel(delta=1)
                time.sleep(0.1)
        else:
            prev_dist = None
            scroll_history.clear()

    cv2.imshow("Camera feed", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

## Contributions

Contributions, issues, and feature requests are welcome! Feel free to check the [issues page](https://github.com/yourusername/Hand-Control-Using-OpenCV-and-Python/issues) if you have any suggestions or questions.

## Acknowledgements

- [OpenCV](https://opencv.org/)
- [cvzone](https://www.cvzone.com/)
- [mouse](https://pypi.org/project/mouse/)
- [keyboard](https://pypi.org/project/keyboard/)

