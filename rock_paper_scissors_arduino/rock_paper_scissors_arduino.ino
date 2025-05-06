#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pulse range for your servo (tweak if needed)
#define SERVO_MIN  150  // Pulse for 0 degrees
#define SERVO_MAX  600  // Pulse for 180 degrees

#define SHOULDER 0  // Channel for shoulder servo
#define ELBOW 1     // Channel for elbow servo
#define FINGERS_R 2 // Channel for finger servo (right fingers)
#define FINGERS_L 3 // Channel for finger servo (right fingers)

// Starting angles
#define SHOULDER_START 0    // TBD
#define ELBOW_START 90      // TBD
#define FINGERS_R_OPEN 0   // TBD 
#define FINGERS_L_OPEN 0   // TBD
// note that fingers_l and fingers_r could be defined by the same global parameters but for now I wanted to fine-tune them individually

// Movement angles
#define SHOULDER_RAISE 90      // Shoulder lifts up
#define ELBOW_UP       110     // Slight lift
#define ELBOW_DOWN     70      // Slight drop
#define FINGERS_R_CLOSED 120
#define FINGERS_L_CLOSED 120

/* --- UTILITY FUNCTIONS --- */
// Map an angle (0Ã¢ÂÂ180) to a pulse length (150Ã¢ÂÂ600)
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

// Simultaneously move three motors smoothly from their respective startN to endN angle
void smoothTripleMovement(
  uint8_t ch1, int start1, int end1, int speed1,
  uint8_t ch2, int start2, int end2, int speed2,
  uint8_t ch3, int start3, int end3, int speed3,
  int stepDelay ) {
  int angle1 = start1;
  int angle2 = start2;
  int angle3 = start3;

  int step1 = (start1 < end1) ? speed1 : -speed1;
  int step2 = (start2 < end2) ? speed2 : -speed2;
  int step3 = (start3 < end3) ? speed3 : -speed3;

  while (angle1 != end1 || angle2 != end2 || angle3 != end3) {
    if (angle1 != end1) {
      setServoAngle(ch1, angle1);
      angle1 += step1;
    }

    if (angle2 != end2) {
      setServoAngle(ch2, angle2);
      angle2 += step2;
    }

    if (angle3 != end3) {
      setServoAngle(ch3, angle3);
      angle3 += step3;
    }

    delay(stepDelay);
  }

  // Final correction to hit exact angles
  setServoAngle(ch1, end1);
  setServoAngle(ch2, end2);
  setServoAngle(ch3, end3);
}


/* --- GAME --- */
void play() {
  // Shoulder raise
  for(int angle = SHOULDER_START; angle <= SHOULDER_RAISE; angle += 1) {
    setServoAngle(SHOULDER, angle);
    delay(20);
  }

  delay(2000);

  // Close all fingers
  smoothDualMovement(FINGERS_R, FINGERS_R_OPEN, FINGERS_R_CLOSED, 1, FINGERS_L, FINGERS_L_OPEN, FINGERS_L_CLOSED, 1, 10);

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
  
  smoothMovement(ELBOW, ELBOW_DOWN, ELBOW_UP, 20);
  delay(100);
  if (choice == 0) {
    Serial.println("ROCK");
    // finger servos do nothing since hand is already closed
    smoothMovement(ELBOW, ELBOW_UP, ELBOW_DOWN, 20);
  } else if (choice == 1) {
    Serial.println("PAPER");
    smoothTripleMovement(ELBOW, ELBOW_UP, ELBOW_DOWN, 1, FINGERS_R, FINGERS_R_CLOSED, FINGERS_R_OPEN, 2, FINGERS_L, FINGERS_L_CLOSED, FINGERS_L_OPEN, 2,  10);
  } else {
    Serial.println("SCISSORS");
    smoothDualMovement(ELBOW, ELBOW_UP, ELBOW_DOWN, 1, FINGERS_L, FINGERS_L_CLOSED, FINGERS_L_OPEN, 2, 10);
  }

  delay(5000);

  // go back to starting position
  if(choice == 0) {
    smoothDualMovement(FINGERS_R, FINGERS_R_CLOSED, FINGERS_R_OPEN, 1, FINGERS_L, FINGERS_L_CLOSED, FINGERS_L_OPEN, 1, 10);
  } else if (choice == 2) {
    smoothMovement(FINGERS_R, FINGERS_R_CLOSED, FINGERS_R_OPEN, 20);
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
  setServoAngle(FINGERS_R, FINGERS_R_OPEN);
  setServoAngle(FINGERS_L, FINGERS_L_OPEN);

  delay(100);
  Serial.println("READY");
}


void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "START") {
      //randomSeed(analogRead(A0) + micros()); // Use current time (microseconds since boot), leave A0 unconnected
      while (Serial.available() == 0);  // Wait for the seed
      long seed = Serial.readStringUntil('\n').toInt();
      randomSeed(seed);

      play();
    }
  }
  else {
    // Keep servos in starting angle
    setServoAngle(SHOULDER, SHOULDER_START);
    setServoAngle(ELBOW, ELBOW_START);
    setServoAngle(FINGERS_R, FINGERS_R_OPEN);
    setServoAngle(FINGERS_L, FINGERS_L_OPEN);
  }
  
  delay(100);
  
  

}