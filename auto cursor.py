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

        # Swipe detection
        if prev_hand_x is not None:
            hand_move_x = hand_center_x - prev_hand_x
            if abs(hand_move_x) > swipe_threshold:
                if hand_move_x > 0:
                    print("Swipe Right")
                else:
                    print("Swipe Left")
            prev_hand_x = hand_center_x
        else:
            prev_hand_x = hand_center_x

        # Trigger Ctrl + C
        if fingers == [0, 1, 1, 1, 1]:
            keyboard.press_and_release('ctrl+c')
            time.sleep(0.5)

        # Trigger Ctrl + V
        if fingers == [1, 1, 1, 1, 1]:
            keyboard.press_and_release('ctrl+v')
            time.sleep(0.5)

    cv2.imshow("Camera feed", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
