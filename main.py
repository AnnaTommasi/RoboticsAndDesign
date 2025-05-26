from OA1 import play_rock_paper_scissors
import serial
import time

actuation1_arduino = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_6493534373335160C051-if00'
actuation2_arduino = 'usb-Arduino__www.arduino.cc__0042_85735313932351818052-if00'

com = serial.Serial('/dev/ttyACM2', 9600)
act_1 = serial.Serial(actuation1_arduino, 9600)
act_2 = serial.Serial(actuation2_arduino, 9600)
time.sleep(2)  # wait for Arduino to reset

# Wait for Arduino to be ready
while True:
        if com.in_waiting > 0:
                msg = com.readline().decode().strip()
                if msg == "READY":
                        break
while True:
        if act_1.in_waiting > 0:
                msg = act_1.readline().decode().strip()
                if msg == "READY":
                        break
while True:
        if act_2.in_waiting > 0:
                msg = act_2.readline().decode().strip()
                if msg == "READY":
                        break
                        
print("All of the arduinos are ready")
com.write(b'START\n')
print("Calling first arduino")
time.sleep(0.05)

while True:
        if com.in_waiting > 0:
                msg = com.readline().decode().strip()
                print(msg)
                if msg == "CALL OA1":
                        print("Calling second arduino")
                        result = play_rock_paper_scissors()
                        print(result)
                 if msg == "CALL OA2":
                        print("Calling the multiplayer game")
                        play_multiplayer_game(com, act_2)
                        #print(result)
