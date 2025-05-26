from OA1 import play_rock_paper_scissors
import serial
import time

actuation1_arduino = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_6493534373335160C051-if00'
act_1 = serial.Serial(actuation1_arduino, 9600)
time.sleep(2)  # wait for Arduino to reset

# Wait for Arduino to be ready
while True:
	if act_1.in_waiting > 0:
		msg = act_1.readline().decode().strip()
		if msg == "READY":
			break

result = play_rock_paper_scissors()
print(result)
time.sleep(0.05)
