from OA1 import play_rock_paper_scissors
import serial
import time
import zmq
import json
import threading

actuation1_arduino = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_6493534373335160C051-if00'
actuation2_arduino = 'usb-Arduino__www.arduino.cc__0042_85735313932351818052-if00'

# ZMQ communication settings
ZMQ_PUBLISHER_IP = "192.168.1.30"  # This device's IP
ZMQ_SUBSCRIBER_IP = "192.168.1.40"  # The other system's IP
COMMAND_PORT = 5556  # Port for sending commands

# Setup ZMQ communication
def setup_zmq_communication():
    context = zmq.Context()
    
    # Create subscriber for tracking data (to receive FOUND messages)
    subscriber = context.socket(zmq.SUB)
    subscriber.connect(f"tcp://{ZMQ_SUBSCRIBER_IP}:{5561}")  # Assuming port 5561 for tracking data
    subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
    print(f"Connected to tracking data channel on {ZMQ_SUBSCRIBER_IP}:5561")
    
    # Create publisher for commands
    publisher = context.socket(zmq.PUB)
    publisher.bind(f"tcp://{ZMQ_PUBLISHER_IP}:{COMMAND_PORT}")
    print(f"Created command publisher on {ZMQ_PUBLISHER_IP}:{COMMAND_PORT}")
    
    return subscriber, publisher

# Function to handle ZMQ messages
def zmq_listener(subscriber, com):
    while True:
        try:
            # Try to receive a message
            message = subscriber.recv_string(flags=zmq.NOBLOCK)
            print(f"Received ZMQ message: {message}")
            
            # Try to parse as JSON first
            try:
                data = json.loads(message)
                # Handle JSON messages here if needed
                if 'found' in data and 'num_people' in data and 'focus_angle' in data:
                    found_msg = f"FOUND {data['num_people']} {data['focus_angle']}"
                    com.write(f"{found_msg}\n".encode())
                    print(f"Forwarded to Arduino: {found_msg}")
            except json.JSONDecodeError:
                # If not JSON, check if it's a direct FOUND message
                if message.startswith("FOUND "):
                    # Forward the message to the Arduino
                    com.write(f"{message}\n".encode())
                    print(f"Forwarded to Arduino: {message}")
        except zmq.Again:
            # No message available
            pass
        except Exception as e:
            print(f"ZMQ Error: {e}")
        
        time.sleep(0.01)  # Short sleep to prevent CPU hogging

# Function to handle multiplayer game
def play_multiplayer_game(com, act_2):
    print("Starting multiplayer game")
    # Add your multiplayer game logic here
    # This is a placeholder function since it was referenced but not defined

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

# Initialize ZMQ communication
subscriber, command_publisher = setup_zmq_communication()

# Start the ZMQ listener thread
zmq_thread = threading.Thread(target=zmq_listener, args=(subscriber, com), daemon=True)
zmq_thread.start()
print("ZMQ communication started")

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
                if msg == "START MOVE":
                        print("Sending SEARCH command via ZMQ")
                        command_publisher.send_string(json.dumps({
                            'command': 'set_state',
                            'state': 'SEARCH'
                        }))
                        print("Command sent")
		# check message queue
        # if people are found: com.write(b'FOUND [NUM_PEOPLE] [FOCUS_ANGLE(0-179)]\n')
        # ex: com.write(b'FOUND 1 90\n')