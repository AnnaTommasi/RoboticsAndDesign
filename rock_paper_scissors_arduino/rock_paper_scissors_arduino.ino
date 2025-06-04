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
#define SHOULDER_START 0
#define ELBOW_START 90
#define FINGER_OPEN 150

// Movement angles
#define SHOULDER_RAISE 65      // Shoulder lifts up
#define ELBOW_UP       90     // Slight lift
#define ELBOW_DOWN     45      // Slight drop
#define ELBOW_PLAYCHOICE 60   // The last time the elbow doesn't drop as much to better show the choice
#define FINGER_CLOSED 40

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

  for (int angle = startAngle;
       (step > 0 && angle < endAngle) || (step < 0 && angle > endAngle);
       angle += step) {
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

  bool done1 = false, done2 = false;

  while (!done1 || !done2) {
    if (!done1) {
      setServoAngle(ch1, angle1);
      angle1 += step1;
      if ((step1 > 0 && angle1 >= end1) || (step1 < 0 && angle1 <= end1)) {
        angle1 = end1;
        done1 = true;
      }
    }

    if (!done2) {
      setServoAngle(ch2, angle2);
      angle2 += step2;
      if ((step2 > 0 && angle2 >= end2) || (step2 < 0 && angle2 <= end2)) {
        angle2 = end2;
        done2 = true;
      }
    }

    delay(stepDelay);
  }

  // Final correction (may be redundant but ensures exact position)
  setServoAngle(ch1, end1);
  setServoAngle(ch2, end2);
}

// Simultaneously move 2 fingers smoothly from start to end angle
void smoothFingerMovement(uint8_t ch1, uint8_t ch2, int start, int end, int stepDelay) {

  int angle = start;
  int step = (start < end) ? 4 : -4;

  while ((step > 0 && angle < end) || (step < 0 && angle > end)) {
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
  int step = (start < end) ? 4 : -4;

  while ((step > 0 && angle < end) || (step < 0 && angle > end)) {
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

  int stepElbow = (ELBOW_UP < ELBOW_PLAYCHOICE) ? 1 : -1;
  int stepFinger = (FINGER_CLOSED < FINGER_OPEN) ? 4 : -4;

  bool doneElbow = false;
  bool doneFinger = false;

  while (!doneElbow || !doneFinger) {
    if (!doneElbow) {
      setServoAngle(ELBOW, angleElbow);
      angleElbow += stepElbow;
      if ((stepElbow > 0 && angleElbow >= ELBOW_PLAYCHOICE) || (stepElbow < 0 && angleElbow <= ELBOW_PLAYCHOICE)) {
        angleElbow = ELBOW_PLAYCHOICE;
        doneElbow = true;
      }
    }

    if (!doneFinger) {
      setServoAngle(FINGER_1, angleFinger);
      setServoAngle(FINGER_2, angleFinger);
      setServoAngle(FINGER_3, angleFinger);
      setServoAngle(FINGER_4, angleFinger);
      angleFinger += stepFinger;
      if ((stepFinger > 0 && angleFinger >= FINGER_OPEN) || (stepFinger < 0 && angleFinger <= FINGER_OPEN)) {
        angleFinger = FINGER_OPEN;
        doneFinger = true;
      }
    }

    delay(5);
  }

  // Final correction
  setServoAngle(ELBOW, ELBOW_PLAYCHOICE);
  setServoAngle(FINGER_1, FINGER_OPEN);
  setServoAngle(FINGER_2, FINGER_OPEN);
  setServoAngle(FINGER_3, FINGER_OPEN);
  setServoAngle(FINGER_4, FINGER_OPEN);
}

// Play scissors choice by moving the elbow down and extending two fingers
void playScissors() {
  int angleElbow = ELBOW_UP;
  int angleFinger = FINGER_CLOSED;

  int stepElbow = (ELBOW_UP < ELBOW_PLAYCHOICE) ? 1 : -1;
  int stepFinger = (FINGER_CLOSED < FINGER_OPEN) ? 4 : -4;

  bool doneElbow = false;
  bool doneFinger = false;

  while (!doneElbow || !doneFinger) {
    if (!doneElbow) {
      setServoAngle(ELBOW, angleElbow);
      angleElbow += stepElbow;
      if ((stepElbow > 0 && angleElbow >= ELBOW_PLAYCHOICE) || (stepElbow < 0 && angleElbow <= ELBOW_PLAYCHOICE)) {
        angleElbow = ELBOW_PLAYCHOICE;
        doneElbow = true;
      }
    }

    if (!doneFinger) {
      setServoAngle(FINGER_1, angleFinger);
      setServoAngle(FINGER_2, angleFinger);
      angleFinger += stepFinger;
      if ((stepFinger > 0 && angleFinger >= FINGER_OPEN) || (stepFinger < 0 && angleFinger <= FINGER_OPEN)) {
        angleFinger = FINGER_OPEN;
        doneFinger = true;
      }
    }

    delay(5);
  }

  // Final correction to exact angles
  setServoAngle(ELBOW, ELBOW_PLAYCHOICE);
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
    if(i!=0)
      smoothMovement(ELBOW, ELBOW_DOWN, ELBOW_UP, 15);
    delay(100);
    smoothMovement(ELBOW, ELBOW_UP, ELBOW_DOWN, 15);
    delay(100);
  }

  // Play choice
  int choice = random(0, 3); // 0 = rock, 1 = paper, 2 = scissors
  
  // Last bounce and display of selected gesture
  smoothMovement(ELBOW, ELBOW_DOWN, ELBOW_UP, 15);
  delay(100);
  if (choice == 0) {
    Serial.println("ROCK");
    // finger servos do nothing since hand is already closed
    smoothMovement(ELBOW, ELBOW_UP, ELBOW_PLAYCHOICE, 15);
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

  smoothDualMovement(SHOULDER, SHOULDER_RAISE, SHOULDER_START, 3, ELBOW, ELBOW_PLAYCHOICE, ELBOW_START, 1, 20);

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
    // Debug code for finger movement
    if (command == "DEBUG") {
      delay(3000);
      smoothFingerMovement(FINGER_1, FINGER_2, FINGER_3, FINGER_4, FINGER_OPEN, FINGER_CLOSED, 5);
      delay(2000);      
      smoothFingerMovement(FINGER_1, FINGER_2, FINGER_3, FINGER_4, FINGER_CLOSED, FINGER_OPEN, 5);
      delay(2000);      
      smoothFingerMovement(FINGER_1, FINGER_2, FINGER_3, FINGER_4, FINGER_OPEN, FINGER_CLOSED, 5);
      delay(2000);      
      smoothFingerMovement(FINGER_1, FINGER_2, FINGER_CLOSED, FINGER_OPEN, 5);
      delay(2000);      
      smoothFingerMovement(FINGER_3, FINGER_4, FINGER_CLOSED, FINGER_OPEN, 5);
    }
  }
  // Keep servos in starting angle
  setServoAngle(SHOULDER, SHOULDER_START);
  setServoAngle(ELBOW, ELBOW_START);
  setServoAngle(FINGER_1, FINGER_OPEN);
  setServoAngle(FINGER_2, FINGER_OPEN);
  setServoAngle(FINGER_3, FINGER_OPEN);
  setServoAngle(FINGER_4, FINGER_OPEN);

  delay(100);
}