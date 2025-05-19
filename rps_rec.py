# Code for testing and debugging rock paper scissors image recognition.
# This python program uses a continuous recording to determine recognition accuracy

import cv2
from HandTrackingModule import HandDetector
import time
import math


cap = cv2.VideoCapture(0)

detector = HandDetector(maxHands=6)

# Define center of focus
focusX = int(480 / 2)
focusY = int(320 / 2)
focusPoint = (focusX, focusY)

def distance(pt1, pt2):
    return math.hypot(pt1[0] - pt2[0], pt1[1] - pt2[1])


while True:
    success, frame = cap.read()
    if not success:
        break
        
    frame = cv2.resize(frame, (480, 320))
    frame = cv2.flip(frame, -1)
    
    hands, frame = detector.findHands(frame)

    playerMove = "No hand detected"
    fingers = [0, 0, 0, 0, 0]

    playerHand = None
    minDist = float("inf")

    # Select hand closest to the focus point
    for hand in hands:
        wrist = hand["lmList"][0][:2]  # Landmark 0 = wrist
        dist = distance(wrist, focusPoint)
        if dist < minDist:
            minDist = dist
            playerHand = hand

    if playerHand:
        fingers = detector.getFingerStateByAngle(playerHand)

        if fingers == [0, 0, 0, 0, 0] or fingers == [1, 0, 0, 0, 0]:
            playerMove = "rock"
        elif fingers == [1, 1, 1, 1, 1] or fingers == [0, 1, 1, 1, 1]:
            playerMove = "paper"
        elif fingers == [0, 1, 1, 0, 0] or fingers == [1, 1, 1, 0, 0]:
            playerMove = "scissors"
        else:
            playerMove = "no valid input"


    # Display current move
    cv2.circle(frame, focusPoint, 5, (255, 0, 255), cv2.FILLED)
    cv2.putText(frame, f"Move: {playerMove}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    #cv2.putText(frame, f"Move: {fingers}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 3)


    cv2.imshow("image", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
        break

cap.release()
cv2.destroyAllWindows()
