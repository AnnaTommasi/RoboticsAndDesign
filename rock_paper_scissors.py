import cv2
from HandTrackingModule import HandDetector
from picamera2 import Picamera2
import time
import serial
import random

debug = 1

# Adjust port based on your device: e.g., '/dev/ttyACM0' or '/dev/ttyUSB0'
ser = serial.Serial('/dev/ttyACM1', 9600)
time.sleep(2)  # wait for Arduino to reset

cam = Picamera2()
config = cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}, buffer_count=2)
cam.configure(config)
cam.start()

playerMove = "INVALID"

while True:
    if ser.in_waiting > 0:
        msg = ser.readline().decode().strip()
        if msg == "READY":
            break

# Send command to start game
ser.write(b'START\n')
time.sleep(0.05)

seed = random.randint(0, 100000)
ser.write(f"{seed}\n".encode())

# Wait for Arduino response
while True:
    if ser.in_waiting > 0:
        robot_choice = ser.readline().decode().strip()

        detector = HandDetector(maxHands=1)
        time.sleep(1)  # wait one second before detecting the player move

        img = cam.capture_array()
        cv2.waitKey(1)  # delay of 1ms

        hands, img = detector.findHands(img)

        if hands:
            hand = hands[0]
            fingers = detector.extendedFingers(hand)
            if fingers == [0, 0, 0, 0, 0] or fingers == [1, 0, 0, 0, 0]:
                playerMove = "ROCK"
            elif fingers == [1, 1, 1, 1, 1]:
                playerMove = "PAPER"
            elif fingers == [0, 1, 1, 0, 0] or fingers == [1, 1, 1, 0, 0]:
                playerMove = "SCISSORS"
            else:
                playerMove = "INVALID"

        if debug == 1:
            cv2.imwrite('rps_outcome_image.jpg', img)

        print(f"Robot played: {robot_choice}, user played: {playerMove}\n")
        outcome = "outcome"
        if playerMove == "INVALID":
            outcome = "couldn't read hand"
        elif playerMove == "ROCK":
            if robot_choice == "ROCK":
                outcome = "TIE"
            elif robot_choice == "PAPER":
                outcome = "WIN"
            else:
                outcome = "LOSE"
        elif playerMove == "PAPER":
            if robot_choice == "ROCK":
                outcome = "LOSE"
            elif robot_choice == "PAPER":
                outcome = "TIE"
            else:
                outcome = "WIN"
        else:
            if robot_choice == "ROCK":
                outcome = "WIN"
            elif robot_choice == "PAPER":
                outcome = "LOSE"
            else:
                outcome = "TIE"

        print(outcome)

        break

cam.stop()
