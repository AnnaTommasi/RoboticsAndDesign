import cv2
from HandTrackingModule import HandDetector
from picamera2 import Picamera2
import time
import math


cam = Picamera2()
config = cam.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"},
    buffer_count=2
)
cam.configure(config)
# Define focus point
frame_width, frame_height = config["main"]["size"]
focusX = int(frame_width / 2)
focusY = int(frame_height / 2)
focusPoint = (focusX, focusY)

cam.start()

detector = HandDetector(maxHands=8)

def distance(pt1, pt2):
    return math.hypot(pt1[0] - pt2[0], pt1[1] - pt2[1])


while True:
    img = cam.capture_array()

    hands, img = detector.findHands(img)

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
        #fingers = detector.fingerFoldAngles(hand, fingerName="Pinky", img=img)

        if fingers == [0, 0, 0, 0, 0] or fingers == [1, 0, 0, 0, 0]:
            playerMove = "rock"
        elif fingers == [1, 1, 1, 1, 1] or fingers == [0, 1, 1, 1, 1]:
            playerMove = "paper"
        elif fingers == [0, 1, 1, 0, 0] or fingers == [1, 1, 1, 0, 0]:
            playerMove = "scissors"
        else:
            playerMove = "no valid input"


    # Display current move
    #cv2.circle(img, focusPoint, 10, (255, 0, 255), cv2.FILLED)
    cv2.putText(img, f"Move: {playerMove}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
    #cv2.putText(img, f"Move: {fingers}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 3)


    cv2.imshow("image", img)
    time.sleep(1);
    if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
        break

cam.stop()
cv2.destroyAllWindows()
