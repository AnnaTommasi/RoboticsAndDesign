# file: OA1.py

import cv2
from HandTrackingModule import HandDetector
from picamera2 import Picamera2
import time
import serial
import random

def play_rock_paper_scissors(arduino_port='/dev/ttyACM1', debug=True):
    ser = serial.Serial(arduino_port, 9600)
    time.sleep(2)  # wait for Arduino to reset

    cam = Picamera2()
    config = cam.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        buffer_count=2
    )
    cam.configure(config)
    cam.start()

    # Send START signal and seed
    ser.write(b'START\n')
    time.sleep(0.05)
    seed = random.randint(0, 100000)
    ser.write(f"{seed}\n".encode())

    # Wait for Arduino move
    while True:
        if ser.in_waiting > 0:
            robot_choice = ser.readline().decode().strip()
            detector = HandDetector(maxHands=1)
            time.sleep(1) # wait one second before detecting the player move

            img = cam.capture_array()
            cv2.waitKey(1)

            hands, img = detector.findHands(img)
            playerMove = "INVALID"

            if hands:
                hand = hands[0]
                fingers = detector.extendedFingers(hand)
                if fingers in ([0, 0, 0, 0, 0], [1, 0, 0, 0, 0]):
                    playerMove = "ROCK"
                elif fingers == [1, 1, 1, 1, 1]:
                    playerMove = "PAPER"
                elif fingers in ([0, 1, 1, 0, 0], [1, 1, 1, 0, 0]):
                    playerMove = "SCISSORS"

            if debug:
                cv2.imwrite('rps_outcome_image.jpg', img)

            # Determine outcome
            if playerMove == "INVALID":
                outcome = "couldn't read hand"
            elif playerMove == robot_choice:
                outcome = "TIE"
            elif (playerMove, robot_choice) in [("ROCK", "SCISSORS"), ("PAPER", "ROCK"), ("SCISSORS", "PAPER")]:
                outcome = "LOSE"
            else:
                outcome = "WIN"

            print(f"Robot played: {robot_choice}, user played: {playerMove}")
            print(f"Outcome: {outcome}")
            cam.stop()

            return {
                "robot": robot_choice,
                "player": playerMove,
                "outcome": outcome
            }
