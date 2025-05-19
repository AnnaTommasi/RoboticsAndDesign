#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pulse range for your servo (tweak if needed)
#define SERVO_MIN  100  // Pulse for 0 degrees (~500us)
#define SERVO_MAX  500  // Pulse for 180 degrees (~2400us)

#define SHOULDER 0  // Channel for shoulder servo
#define ELBOW 1     // Channel for elbow servo
#define FINGER_1 4 // Channel for finger servo ("index")
#define FINGER_2 5 // Channel for finger servo ("medium")
#define FINGER_3 6 // Channel for finger servo ("ring")
#define FINGER_4 7 // Channel for finger servo ("pinky")

// Starting angles
#define SHOULDER_START 0    // TBD
#define ELBOW_START 90      // TBD
#define FINGER_OPEN 150   // TBD 

// Movement angles
#define SHOULDER_RAISE 60      // Shoulder lifts up
#define ELBOW_UP       110     // Slight lift
#define ELBOW_DOWN     70      // Slight drop
#define FINGER_CLOSED 30

/* --- UTILITY FUNCTIONS --- */
// Map an angle to a pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

// Set servo angle on a given channel
void setServoAngle(uint8_t channel, int angle) {
  int pulse = angleToPulse(angle);
  pwm.setPWM(channel, 0, pulse);
}

// Move a motor smoothly from startAngle to endAngle
void smoothMovement(uint8_t channel, int startAngle, int endAngle, int stepDelay) {
  int step = (startAngle < endAngle) ? 2 : -2;

  for (int angle = startAngle; angle != endAngle; angle += step) {
    setServoAngle(channel, angle);
    delay(stepDelay); // Delay per step, in ms
  }

  setServoAngle(channel, endAngle); // Make sure it hits final angle exactly
}

// Simultaneously move two motors smoothly from their respective startN to endN angle
void smoothDualMovement(uint8_t ch1, int start1, int end1, int speed1,
                        uint8_t ch2, int start2, int end2, int speed2,
                        int stepDelay) {
  int angle1 = start1;
  int angle2 = start2;

  int step1 = (start1 < end1) ? speed1 : -speed1;
  int step2 = (start2 < end2) ? speed2 : -speed2;

  while (angle1 != end1 || angle2 != end2) {
    if (angle1 != end1) {
      setServoAngle(ch1, angle1);
      angle1 += step1;
    }

    if (angle2 != end2) {
      setServoAngle(ch2, angle2);
      angle2 += step2;
    }

    delay(stepDelay);
  }

  // Make sure both servos land exactly at final positions
  setServoAngle(ch1, end1);
  setServoAngle(ch2, end2);
}

// Simultaneously move 2 fingers smoothly from start to end angle
void smoothFingerMovement(uint8_t ch1, uint8_t ch2, int start, int end, int stepDelay) {

  int angle = start;
  int step = (start < end) ? 2 : -2;

  while (angle != end) {
    setServoAngle(ch1, angle);
    setServoAngle(ch2, angle);
    angle += step;

    delay(stepDelay);
  }

  // Make sure both servos land exactly at final positions
  setServoAngle(ch1, end);
  setServoAngle(ch2, end);
}

// Simultaneously move all 4 fingers smoothly from start to end angle
void smoothFingerMovement(uint8_t ch1, uint8_t ch2, uint8_t ch3, uint8_t ch4, int start, int end, int stepDelay) {

  int angle = start;
  int step = (start < end) ? 2 : -2;

  while (angle != end) {
    setServoAngle(ch1, angle);
    setServoAngle(ch2, angle);
    setServoAngle(ch3, angle);
    setServoAngle(ch4, angle);
    angle += step;

    delay(stepDelay);
  }

  // Make sure both servos land exactly at final positions
  setServoAngle(ch1, end);
  setServoAngle(ch2, end);
  setServoAngle(ch3, end);
  setServoAngle(ch4, end);
}

// Play paper choice by moving the elbow down and extending all fingers
void playPaper() {
  int angleElbow = ELBOW_UP;
  int angleFinger = FINGER_CLOSED;

  int stepElbow = (ELBOW_UP < ELBOW_DOWN) ? 1 : -1;
  int stepFinger = (FINGER_CLOSED < FINGER_OPEN) ? 4 : -4;

  while (angleElbow != ELBOW_DOWN || angleFinger != FINGER_OPEN) {
    if (angleElbow != ELBOW_DOWN) {
      setServoAngle(ELBOW, angleElbow);
      angleElbow += stepElbow;
    }
    if (angleFinger != FINGER_OPEN) {
      setServoAngle(FINGER_1, angleFinger);
      setServoAngle(FINGER_2, angleFinger);
      setServoAngle(FINGER_3, angleFinger);
      setServoAngle(FINGER_4, angleFinger);
      angleFinger += stepFinger;
    }

    delay(5);
  }

  // Final correction to hit exact angles
  setServoAngle(ELBOW, ELBOW_DOWN);
  setServoAngle(FINGER_1, FINGER_OPEN);
  setServoAngle(FINGER_2, FINGER_OPEN);
  setServoAngle(FINGER_3, FINGER_OPEN);
  setServoAngle(FINGER_4, FINGER_OPEN);
}

// Play scissors choice by moving the elbow down and extending two fingers
void playScissors() {
  int angleElbow = ELBOW_UP;
  int angleFinger = FINGER_CLOSED;

  int stepElbow = (ELBOW_UP < ELBOW_DOWN) ? 1 : -1;
  int stepFinger = (FINGER_CLOSED < FINGER_OPEN) ? 4 : -4;

  while (angleElbow != ELBOW_DOWN || angleFinger != FINGER_OPEN) {
    if (angleElbow != ELBOW_DOWN) {
      setServoAngle(ELBOW, angleElbow);
      angleElbow += stepElbow;
    }
    if (angleFinger != FINGER_OPEN) {
      setServoAngle(FINGER_1, angleFinger);
      setServoAngle(FINGER_2, angleFinger);
      angleFinger += stepFinger;
    }

    delay(5);
  }

  // Final correction to hit exact angles
  setServoAngle(ELBOW, ELBOW_DOWN);
  setServoAngle(FINGER_1, FINGER_OPEN);
  setServoAngle(FINGER_2, FINGER_OPEN);
}



/* --- GAME --- */
void play() {
  // Shoulder raise
  smoothMovement(SHOULDER, SHOULDER_START, SHOULDER_RAISE, 20);

  delay(2000);

  // Close all fingers
  smoothFingerMovement(FINGER_1, FINGER_2, FINGER_3, FINGER_4, FINGER_OPEN, FINGER_CLOSED, 10);

  // Elbow bounce
  for (int i = 0; i < 2; i++) {
    if(i==0)
      smoothMovement(ELBOW, ELBOW_START, ELBOW_UP, 20);
    else
      smoothMovement(ELBOW, ELBOW_DOWN, ELBOW_UP, 20);
    delay(100);
    smoothMovement(ELBOW, ELBOW_UP, ELBOW_DOWN, 20);
    delay(100);
  }

  // Play choice
  int choice = random(0, 3); // 0 = rock, 1 = paper, 2 = scissors
  
  // Last bounce and display of selected gesture
  smoothMovement(ELBOW, ELBOW_DOWN, ELBOW_UP, 20);
  delay(100);
  if (choice == 0) {
    Serial.println("ROCK");
    // finger servos do nothing since hand is already closed
    smoothMovement(ELBOW, ELBOW_UP, ELBOW_DOWN, 20);
  } else if (choice == 1) {
    Serial.println("PAPER");
    playPaper();
  } else {
    Serial.println("SCISSORS");
    playScissors();
  }

  delay(5000);

  // go back to starting position
  if(choice == 0) {
    smoothFingerMovement(FINGER_1, FINGER_2, FINGER_3, FINGER_4, FINGER_CLOSED, FINGER_OPEN, 10);
  } else if (choice == 2) {
    smoothFingerMovement(FINGER_3, FINGER_4, FINGER_CLOSED, FINGER_OPEN, 10);
  }

  smoothDualMovement(SHOULDER, SHOULDER_RAISE, SHOULDER_START, 3, ELBOW, ELBOW_DOWN, ELBOW_START, 1, 20);

}

/* --- SETUP --- */
void setup() {
  Serial.begin(9600);
  Serial.flush();
  pwm.begin();
  pwm.setPWMFreq(50); // 50 Hz for servos

  // Set servos to starting angle
  setServoAngle(SHOULDER, SHOULDER_START);
  setServoAngle(ELBOW, ELBOW_START);
  setServoAngle(FINGER_1, FINGER_OPEN);
  setServoAngle(FINGER_2, FINGER_OPEN);
  setServoAngle(FINGER_3, FINGER_OPEN);
  setServoAngle(FINGER_4, FINGER_OPEN);

  delay(100);
  Serial.println("READY");
}


void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "START") {
      while (Serial.available() == 0);  // Wait for the seed
      long seed = Serial.readStringUntil('\n').toInt();
      randomSeed(seed);

      play();
    }
  }
  // Keep servos in starting angle
  setServoAngle(SHOULDER, SHOULDER_START);
  setServoAngle(ELBOW, ELBOW_START);
  setServoAngle(FINGER_1, FINGER_OPEN);
  setServoAngle(FINGER_2, FINGER_OPEN);
  setServoAngle(FINGER_3, FINGER_OPEN);
  setServoAngle(FINGER_4, FINGER_OPEN);


  //INSTRUCTIONS FOR FINE-TUNING (single) MOTOR LIMITS
  // uncomment the line for the motor you are testing, set the appropriate angle and check. I suggest having only one uncommented line at a time to test individual movements
  // If you found the correct angle, go up to the define section (line 22) and set the angle as under "movement angles"
  // In the next lines, I usually started by setting a lower angle than the one in the define section, so remember checking in the end
  delay(2000);
  // DRIVER PIN 0
  // SHOULDER_RAISE: how much the shoulder raises from starting position.
  // (Uncomment next line for testing shoulder)
  //setServoAngle(SHOULDER, 30);    // CHANGE ANGLE VALUE TO TUNE

  // DRIVER PIN 1
  // ELBOW_UP: angle at which the forearm is up. Note that starting angle for elbow is 90, so ELBOW_UP and ELBOW_DOWN will be values around 80 and 100.
  // UP and DOWN angle do not necessarily need to be UP > DOWN, the code should already account for both scenarios
  // (Uncomment next line for testing elbow_up)
  //setServoAngle(ELBOW, 100);    // CHANGE ANGLE VALUE TO TUNE. If values > 90 turn the elbow down, use values < 90

  // (Uncomment next line for testing elbow_down)
  //setServoAngle(ELBOW, 80);    // CHANGE ANGLE VALUE TO TUNE. If values < 90 turn the elbow up, use values > 90

  // DRIVER PIN 4 (index), 5 (middle), 6 (ring), 7 (pinky)
  // FINGER_CLOSED: at what angle the finger is completely closed.
  // (Uncomment next line for testing finger_N)
  //setServoAngle(FINGER_1, 0);    // CHANGE ANGLE VALUE TO TUNE
  //setServoAngle(FINGER_2, 90);    // CHANGE ANGLE VALUE TO TUNE
  //setServoAngle(FINGER_3, 90);    // CHANGE ANGLE VALUE TO TUNE
  //setServoAngle(FINGER_4, 90);    // CHANGE ANGLE VALUE TO TUNE
/*
  smoothMovement(FINGER_1, FINGER_OPEN, FINGER_CLOSED, 5);
  delay(1000);
  smoothMovement(FINGER_1, FINGER_CLOSED, FINGER_OPEN, 5);

  delay(3000);

*/
  // To test whether the whole sequence works: comment all test instructions above (lines 278-305). Open serial monitor (under "tools").
  // When serial monitor prints "READY", write "START". Then input a random number (e.g., 123) and the game should start.

  
  delay(100);
}