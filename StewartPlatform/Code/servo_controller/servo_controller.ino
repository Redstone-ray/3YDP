#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>

// Create PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// WS2812B LED configuration
#define LED_PIN 6          // Digital pin connected to the NeoPixels
#define NUM_LEDS 60        // Total number of LEDs in the strip
#define FIRST_LED_INDEX 3  // Skip first 3 LEDs, start from index 3
#define ACTIVE_LEDS 57     // Number of LEDs actually used (60 - 3)

// Create NeoPixel object
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Servo configuration
const int SERVO1_PORT = 0;
const int SERVO2_PORT = 1;
const int SERVO3_PORT = 2;

const int MIN_ANGLE = 76;   // Top most position
const int MAX_ANGLE = 155;  // Bottom most position
const int NEUTRAL_ANGLE = 128;  // Flat

// Servo pulse length settings for 300° servo (500μS ~ 2500μS)
// At 50Hz (20ms period), 4096 counts per period
// 500μS = (500/20000) * 4096 ≈ 102 counts
// 2500μS = (2500/20000) * 4096 ≈ 512 counts
#define SERVOMIN  102  // Minimum pulse length (500μS)
#define SERVOMAX  512  // Maximum pulse length (2500μS)

// Communication variables
int targetAngles[3] = {NEUTRAL_ANGLE, NEUTRAL_ANGLE, NEUTRAL_ANGLE};
bool newCommand = false;

// LED communication protocol
#define LED_COMMAND_BYTE 250  // Special byte to indicate LED command follows
#define LED_MODE_IDLE 0
#define LED_MODE_CALIBRATING 1
#define LED_MODE_RUNNING_BALANCED 2
#define LED_MODE_RUNNING_BALL 3

// LED state variables
uint8_t ledMode = LED_MODE_IDLE;
uint8_t ledBallPosition = 0;  // LED index for ball position (0-56)
unsigned long lastLedUpdate = 0;
uint16_t ledAnimationStep = 0;
bool blinkState = false;

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
  
  // Initialize NeoPixel strip
  strip.begin();
  strip.show();  // Initialize all pixels to 'off'
  strip.setBrightness(50);  // Set brightness to 50/255 (about 20%)
  
  // Initialize variables
  targetAngles[0] = NEUTRAL_ANGLE;
  targetAngles[1] = NEUTRAL_ANGLE;
  targetAngles[2] = NEUTRAL_ANGLE;
  
  // Start with idle LED mode
  ledMode = LED_MODE_IDLE;
  
  // Optional: Send ready message
  Serial.println("Arduino servo controller ready (I2C PWM Driver + WS2812B LEDs). Echo off.");
}

void loop() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    int firstByte = Serial.read();
    
    // Check if this is an LED command
    if (firstByte == LED_COMMAND_BYTE) {
      // Wait for mode byte with timeout
      unsigned long startWait = millis();
      while (Serial.available() < 1 && (millis() - startWait) < 100) {
        delayMicroseconds(100);
      }
      
      if (Serial.available() >= 1) {
        uint8_t mode = Serial.read();
        
        // Process LED command based on mode
        if (mode == LED_MODE_IDLE) {
          ledMode = LED_MODE_IDLE;
        } else if (mode == LED_MODE_CALIBRATING) {
          ledMode = LED_MODE_CALIBRATING;
        } else if (mode == LED_MODE_RUNNING_BALANCED) {
          ledMode = LED_MODE_RUNNING_BALANCED;
        } else if (mode == LED_MODE_RUNNING_BALL) {
          // Wait for ball position byte with timeout
          startWait = millis();
          while (Serial.available() < 1 && (millis() - startWait) < 100) {
            delayMicroseconds(100);
          }
          if (Serial.available() >= 1) {
            ledBallPosition = Serial.read();
            ledMode = LED_MODE_RUNNING_BALL;
          }
        }
      }
    } 
    // Otherwise, this is a servo command (expect 2 more bytes)
    else if (firstByte >= MIN_ANGLE && firstByte <= MAX_ANGLE) {
      // Wait for remaining 2 bytes with timeout
      unsigned long startWait = millis();
      while (Serial.available() < 2 && (millis() - startWait) < 100) {
        delayMicroseconds(100);
      }
      
      if (Serial.available() >= 2) {
        int angle0 = firstByte;  // First byte for port 0
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
  }
  
  // Update LED animations
  updateLEDs();
}

// Helper function to set color for an active LED index (0-56)
void setActiveLED(uint8_t index, uint32_t color) {
  if (index < ACTIVE_LEDS) {
    strip.setPixelColor(FIRST_LED_INDEX + index, color);
  }
}

// Helper function to clear all active LEDs
void clearActiveLEDs() {
  for (int i = 0; i < ACTIVE_LEDS; i++) {
    setActiveLED(i, 0);
  }
}

// Update LED animations based on current mode
void updateLEDs() {
  unsigned long currentTime = millis();
  
  // Update animation at different rates based on mode
  unsigned long updateInterval = 50;  // Default 50ms (20 Hz)
  
  switch (ledMode) {
    case LED_MODE_IDLE:
      // Blue circling animation - update every 50ms
      if (currentTime - lastLedUpdate >= updateInterval) {
        clearActiveLEDs();
        
        // Main LED position (cycles through all 57 LEDs)
        uint8_t mainLED = ledAnimationStep % ACTIVE_LEDS;
        
        // Set main LED to full brightness blue
        setActiveLED(mainLED, strip.Color(0, 0, 255));
        
        // Set adjacent LEDs to dimmer blue
        uint8_t prevLED = (mainLED - 1 + ACTIVE_LEDS) % ACTIVE_LEDS;
        uint8_t nextLED = (mainLED + 1) % ACTIVE_LEDS;
        setActiveLED(prevLED, strip.Color(0, 0, 80));  // Dimmer
        setActiveLED(nextLED, strip.Color(0, 0, 80));  // Dimmer
        
        strip.show();
        ledAnimationStep++;
        lastLedUpdate = currentTime;
      }
      break;
      
    case LED_MODE_CALIBRATING:
      // Blink red every 4th LED - update every 500ms
      updateInterval = 500;
      if (currentTime - lastLedUpdate >= updateInterval) {
        clearActiveLEDs();
        
        if (blinkState) {
          // Turn on every 4th LED in red
          for (int i = 0; i < ACTIVE_LEDS; i += 4) {
            setActiveLED(i, strip.Color(255, 0, 0));
          }
        }
        // else: LEDs stay off
        
        strip.show();
        blinkState = !blinkState;
        lastLedUpdate = currentTime;
      }
      break;
      
    case LED_MODE_RUNNING_BALANCED:
      // Steady green every 4th LED - only update once when entering this mode
      if (currentTime - lastLedUpdate >= 100) {  // Debounce mode changes
        clearActiveLEDs();
        
        // Turn on every 4th LED in green
        for (int i = 0; i < ACTIVE_LEDS; i += 4) {
          setActiveLED(i, strip.Color(0, 255, 0));
        }
        
        strip.show();
        lastLedUpdate = currentTime;
      }
      break;
      
    case LED_MODE_RUNNING_BALL:
      // Orange LED at ball position - update immediately when position changes
      if (currentTime - lastLedUpdate >= 20) {  // Limit to 50 Hz max
        clearActiveLEDs();
        
        // Ensure ball position is within valid range
        if (ledBallPosition < ACTIVE_LEDS) {
          // Set main LED to orange
          setActiveLED(ledBallPosition, strip.Color(255, 100, 0));
          
          // Set adjacent LEDs to dimmer orange
          uint8_t prevLED = (ledBallPosition - 1 + ACTIVE_LEDS) % ACTIVE_LEDS;
          uint8_t nextLED = (ledBallPosition + 1) % ACTIVE_LEDS;
          setActiveLED(prevLED, strip.Color(100, 40, 0));  // Dimmer
          setActiveLED(nextLED, strip.Color(100, 40, 0));  // Dimmer
        }
        
        strip.show();
        lastLedUpdate = currentTime;
      }
      break;
  }
}