# Robotics and Design
Project of the XIII edition of the Robotics and Design Course, A.Y. 2024-2025.
## Module Outdoor Actuation 1
- **Feugap Esaie**, school of design
- **Harutyunyan Viktorya**, school of design
- **Guza Jennifer**, school of design
- **Tommasi Anna**, school of industrial and information engineering

## Project Specifications
**BB2** is a pet robot that patrols the outdoor area of Bovisa, in front of building B2. As the campus mascot, BB2 actively looks for people to interact with, challenging them to fun games like *Rock-Paper-Scissors* or *Miscela*.

Our team was responsible for designing and building a robotic arm that plays ***Rock-Paper-Scissors*** with students and recognizes their hand gestures in real time. The game results are then processed by the Communication Module, which controls the robot's reactions and responses.

This repository contains all the code developed and implemented for our module.

## Informatics
### System Requirements
The hand-tracking program uses Google's MediaPipe library to detect hands in the video stream and extract hand landmarks. mediaPipe requires **Python 3.11** or lower to function correctly.

To set up the environment manually, install the following packages:
```
pip install opencv-python
pip install cvzone
pip install mediapipe
pip install numpy
pip install pyserial
```

If you're using the provided **Raspberry Pi**, all required packages are already installed in a virtual environment. To activate it, run the following command in the terminal:
```
source venv/bin/activate
```
You can then run any python script inside the virtual environment:
```
python test_OA1.py
```

### Delivered files
- **[rock_paper_scissors_arduino](https://github.com/AnnaTommasi/RoboticsAndDesign/tree/main/rock_paper_scissors_arduino)** - This folder contains the Arduino code responsible for controlling the robotic arm via dedicated servo motors.
To initiate a game round, the Arduino program expects two separate serial commands:
    - A string ```START``` to signal the beginning of the game.
    - A random number to be used as the seed for generating the robot's move.
- **[OA1.py](https://github.com/AnnaTommasi/RoboticsAndDesign/blob/main/OA1.py)** - This file contains the main function ```play_rock_paper_scissors()``` that runs the Rock-Paper-Scissors interaction between the user and the robotic arm.
    - Activates the camera to recognize the player's hand gesture.
    - Sends a ```START``` signal and a random seed to the Arduino via serial connection to initialize the robot's move.
    - Collects gesture data from the user over a short time window.
    - Uses a ranking mechanism over valid frames to determine the player's final move.
    - Compares the player's move with the robot's and returns the outcome.
- **[HandTrackingModule.py](https://github.com/AnnaTommasi/RoboticsAndDesign/blob/main/HandTrackingModule.py)** - This module provides real-time hand detection and landmark analysis. It was originally developed by Computer Vision Zone, but significant modifications were made to support gesture recognition and gameplay logic for the ```play_rock_paper_scissors()``` function. 
The script includes:
    - **Final game logic utilities**:
        - ```getFingerStateByAngle()```: determines finger states (open or closed) using geometric joint angles.
        - ```fingerFoldAngles()```: calculates fold angles for each finger based on key landmarks. Also includes optional visual debigging tools for tuning angle thresholds interactively.
    - **Experimental/legacy functions**:
        - ```extendedFingers()```: an early method for detecting extended fingers, which was later discarded in favor of the more robust angle-based logic.
- **[test_OA1.py](https://github.com/AnnaTommasi/RoboticsAndDesign/blob/main/test_OA1.py)** - This script was used to independently test the *actuation module* for the robot, bypassing the *communication* interface that would ultimately handle the game execution in the final integrated system.
    - Establishes serial communication with the Arduino controlling the actuators.
    - Waits for a ```READY``` signal from the Arduino before proceeding.
    - Runs the ```play_rock_paper_scissors()``` function imported from ```OA1```

    This setup allows debugging and validation of the actuation pipeline, separate from other modules.
- **[run_rps.sh](https://github.com/AnnaTommasi/RoboticsAndDesign/blob/main/run_rps.sh)** - Convenience script to run the Rock-Paper-Scissors game inside a Python virtual environment.
- **[rps_rec.py](https://github.com/AnnaTommasi/RoboticsAndDesign/blob/main/rps_rec.py)** - This script was used for testing and debugging hand gesture recognition for the Rock-Paper-Scissors game logic, using continuous webcam input. It was used to evaluate and fine-tune the gesture classification logic by:
    - Continuously capturing hand data.
    - Classifying gestures based on finger state angles
    Displaying real-time feedback on-screen for debugging and accuracy assessment.
