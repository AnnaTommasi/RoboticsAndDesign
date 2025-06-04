# file: OA1.py

import cv2
from HandTrackingModule import HandDetector
import time
import serial
import random
import math

actuation1_arduino = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_6493534373335160C051-if00'

def play_rock_paper_scissors(arduino_port=actuation1_arduino):
    ser = serial.Serial(arduino_port, 9600)
    time.sleep(2)  # wait for Arduino to reset
    
    cam = cv2.VideoCapture(0)
    detector = HandDetector(maxHands=2)


    # Define focus point
    frame_width = 480
    frame_height = 320
    focusX = int(frame_width / 2)
    focusY = int(frame_height / 2)
    focusPoint = (focusX, focusY)

    def distance(pt1, pt2):
        return math.hypot(pt1[0] - pt2[0], pt1[1] - pt2[1])


    # Send START signal and seed to Arduino
    ser.write(b'START\n')
    time.sleep(0.05)
    seed = random.randint(0, 100000)
    ser.write(f"{seed}\n".encode())

    # Wait for Arduino move
    while True:
        if ser.in_waiting > 0:
            robot_choice = ser.readline().decode().strip()
            
            print(f"----------START----------")

            time.sleep(0.5) # wait half a second before detecting the player move

            gesture_counts = {"ROCK": 0, "PAPER": 0, "SCISSORS": 0}
            valid_frames = 0
            invalid_frames = 0
            capture_duration = 3  # seconds
            start_time = time.time()

            while time.time() - start_time < capture_duration:
                success, frame = cam.read()
                if not success:
                    break
                    
                # resize (and flip) frame for better image recognition performance
                frame = cv2.resize(frame, (480, 320))
                frame = cv2.flip(frame, -1)
                
                hands, frame = detector.findHands(frame)

                playerMove = "INVALID"
                playerHand = None
                minDist = float("inf")

                # select hand closest to the focus point
                for hand in hands:
                    wrist = hand["lmList"][0][:2]
                    dist = distance(wrist, focusPoint)
                    if dist < minDist:
                        minDist = dist
                        playerHand = hand

                if playerHand:
                    fingers = detector.getFingerStateByAngle(playerHand)

                    if fingers == [0, 0, 0, 0, 0] or fingers == [1, 0, 0, 0, 0]:
                        playerMove = "ROCK"
                    elif fingers == [1, 1, 1, 1, 1] or fingers == [0, 1, 1, 1, 1]:
                        playerMove = "PAPER"
                    elif fingers == [0, 1, 1, 0, 0] or fingers == [1, 1, 1, 0, 0]:
                        playerMove = "SCISSORS"
                    
                    if playerMove != "INVALID":
                        gesture_counts[playerMove] += 1
                        valid_frames += 1
                    
                    else:
                        invalid_frames += 1

                cv2.waitKey(1)
                

            # Determine final move based on frequency
            if valid_frames == 0:
                final_playerMove = "INVALID"
            else:
                final_playerMove = max(gesture_counts, key=gesture_counts.get)

            # Determine outcome
            if final_playerMove == "INVALID":
                outcome = "INVALID"
            elif final_playerMove == robot_choice:
                outcome = "TIE"
            elif (final_playerMove, robot_choice) in [("ROCK", "SCISSORS"), ("PAPER", "ROCK"), ("SCISSORS", "PAPER")]:
                outcome = "LOSE"
            else:
                outcome = "WIN"

            print(f"Robot played: {robot_choice}, user played: {final_playerMove}")
            #print(gesture_counts)
            #print(invalid_frames)
            print(f"Outcome: {outcome}")
            
            break
            
    cam.release()
    return outcome
