from OA1 import play_rock_paper_scissors
import serial
import time

act_1 = serial.Serial('/dev/ttyACM1', 9600)
com = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)  # wait for Arduino to reset

# Wait for Arduino to be ready
while True:
	if act_1.in_waiting > 0:
		msg = act_1.readline().decode().strip()
		if msg == "READY":
			break
while True:
	if com.in_waiting > 0:
		msg = com.readline().decode().strip()
		if msg == "READY":
			break

print("both arduinos ready")
com.write(b'START\n')
print("Calling first arduino")
time.sleep(0.05)

if com.in_waiting > 0:
	msg = com.readline().decode().strip()
	print(msg)
	if msg == "CALL OA1":
		print("Calling second arduino")
		result = play_rock_paper_scissors()
		print(result)
