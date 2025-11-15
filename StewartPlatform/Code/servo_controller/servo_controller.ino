#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo configuration
const int SERVO1_PORT = 0;
const int SERVO2_PORT = 1;
const int SERVO3_PORT = 2;

const int MIN_ANGLE = 0;
const int MAX_ANGLE = 300;  // 300° range of motion
const int NEUTRAL_ANGLE = 150;  // Neutral at center of 300° range

// Servo pulse length settings for 300° servo (500μS ~ 2500μS)
// At 50Hz (20ms period), 4096 counts per period
// 500μS = (500/20000) * 4096 ≈ 102 counts
// 2500μS = (2500/20000) * 4096 ≈ 512 counts
#define SERVOMIN  102  // Minimum pulse length (500μS)
#define SERVOMAX  512  // Maximum pulse length (2500μS)

// Communication variables
int targetAngles[3] = {NEUTRAL_ANGLE, NEUTRAL_ANGLE, NEUTRAL_ANGLE};
bool newCommand = false;

// Helper function to convert angle to PWM pulse length
uint16_t angleToPulse(int angle) {
  // Map angle (0-300) to pulse length (SERVOMIN-SERVOMAX)
  // 300° servo with 500μS to 2500μS pulse width range
  return map(angle, 0, 300, SERVOMIN, SERVOMAX);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize PWM driver
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  
  // Wait for oscillator to stabilize
  delay(10);
  
  // Move all servos to neutral position
  uint16_t neutralPulse = angleToPulse(NEUTRAL_ANGLE);
  pwm.setPWM(SERVO1_PORT, 0, neutralPulse);
  pwm.setPWM(SERVO2_PORT, 0, neutralPulse);
  pwm.setPWM(SERVO3_PORT, 0, neutralPulse);
  
  // Initialize variables
  targetAngles[0] = NEUTRAL_ANGLE;
  targetAngles[1] = NEUTRAL_ANGLE;
  targetAngles[2] = NEUTRAL_ANGLE;
  
  // Optional: Send ready message
  Serial.println("Arduino servo controller ready (I2C PWM Driver). Echo off.");
}

void loop() {
  // Check for incoming serial data - expect 3 bytes
  if (Serial.available() >= 3) {
    // Read three angle bytes
    int angle0 = Serial.read();  // First byte for port 0
    int angle1 = Serial.read();  // Second byte for port 1
    int angle2 = Serial.read();  // Third byte for port 2
    
    // Validate angle ranges and update targets
    bool validCommand = true;
    
    if (angle0 >= MIN_ANGLE && angle0 <= MAX_ANGLE) {
      targetAngles[0] = angle0;
    } else {
      validCommand = false;
    }
    
    if (angle1 >= MIN_ANGLE && angle1 <= MAX_ANGLE) {
      targetAngles[1] = angle1;
    } else {
      validCommand = false;
    }
    
    if (angle2 >= MIN_ANGLE && angle2 <= MAX_ANGLE) {
      targetAngles[2] = angle2;
    } else {
      validCommand = false;
    }
    
    if (validCommand) {
      newCommand = true;
    }
  }
  
  // Update servos if new command received
  if (newCommand) {
    uint16_t pulse0 = angleToPulse(targetAngles[0]);
    uint16_t pulse1 = angleToPulse(targetAngles[1]);
    uint16_t pulse2 = angleToPulse(targetAngles[2]);
    
    pwm.setPWM(SERVO1_PORT, 0, pulse0);
    pwm.setPWM(SERVO2_PORT, 0, pulse1);
    pwm.setPWM(SERVO3_PORT, 0, pulse2);
    newCommand = false;
    
    // Optional: Echo back the angles for debugging
    // Serial.print("Angles set to: ");
    // Serial.print(targetAngles[0]); Serial.print(", ");
    // Serial.print(targetAngles[1]); Serial.print(", ");
    // Serial.println(targetAngles[2]);
  }
}