#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <FastLED.h>

// Create PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// LED configuration
#define LED_PIN     9       // Data pin for WS2812B
#define NUM_LEDS    60      // Total number of LEDs
#define LED_SKIP    3       // Skip first 3 LEDs
#define BRIGHTNESS  100     // LED brightness (0-255)

CRGB leds[NUM_LEDS];

// LED animation state
enum LEDMode {
  MODE_IDLE = 0,        // Blue circling animation
  MODE_CALIBRATION = 1, // Red blinking
  MODE_RUNNING = 2,     // Orange indicator at ball position
  MODE_BALANCED = 3     // Green steady every 4th LED
};

LEDMode currentMode = MODE_IDLE;
uint8_t targetLED = 0;           // For MODE_RUNNING
unsigned long lastAnimUpdate = 0;
uint8_t animStep = 0;            // Animation step counter
bool blinkState = false;         // For blinking animations

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

// Helper function to convert angle to PWM pulse length
uint16_t angleToPulse(int angle) {
  // Map angle (0-300) to pulse length (SERVOMIN-SERVOMAX)
  // 300° servo with 500μS to 2500μS pulse width range
  return map(angle, 0, 300, SERVOMIN, SERVOMAX);
}

// LED Animation Functions
void updateLEDAnimation() {
  unsigned long currentMillis = millis();
  
  switch (currentMode) {
    case MODE_IDLE:
      // Blue circling animation (updates every 50ms)
      if (currentMillis - lastAnimUpdate > 50) {
        lastAnimUpdate = currentMillis;
        FastLED.clear();
        
        // Main LED (bright blue)
        uint8_t mainLED = LED_SKIP + (animStep % (NUM_LEDS - LED_SKIP));
        leds[mainLED] = CRGB(0, 0, 255);
        
        // Adjacent LEDs (dimmer blue)
        uint8_t prev = LED_SKIP + ((animStep - 1 + (NUM_LEDS - LED_SKIP)) % (NUM_LEDS - LED_SKIP));
        uint8_t next = LED_SKIP + ((animStep + 1) % (NUM_LEDS - LED_SKIP));
        leds[prev] = CRGB(0, 0, 100);
        leds[next] = CRGB(0, 0, 100);
        
        FastLED.show();
        animStep++;
      }
      break;
      
    case MODE_CALIBRATION:
      // Red blinking every 4th LED (500ms interval)
      if (currentMillis - lastAnimUpdate > 500) {
        lastAnimUpdate = currentMillis;
        blinkState = !blinkState;
        
        FastLED.clear();
        if (blinkState) {
          for (int i = LED_SKIP; i < NUM_LEDS; i += 4) {
            leds[i] = CRGB(255, 0, 0);
          }
        }
        FastLED.show();
      }
      break;
      
    case MODE_RUNNING:
      // Orange indicator at ball position with neighbors (static until position updates)
      // No animation update needed - updated when new command received
      break;
      
    case MODE_BALANCED:
      // Green steady every 4th LED (static)
      // No animation update needed - set once when mode changes
      break;
  }
}

void setLEDMode(LEDMode mode, uint8_t data = 0) {
  // Only update if mode changed or data changed (for MODE_RUNNING)
  if (currentMode != mode || (mode == MODE_RUNNING && targetLED != data)) {
    currentMode = mode;
    targetLED = data;
    animStep = 0;
    lastAnimUpdate = millis();
    
    // Set static modes immediately
    if (mode == MODE_RUNNING) {
      FastLED.clear();
      // Ensure LED index is valid
      if (targetLED >= LED_SKIP && targetLED < NUM_LEDS) {
        // Main LED (orange)
        leds[targetLED] = CRGB(255, 100, 0);
        
        // Adjacent LEDs (dimmer orange)
        int prev = targetLED - 1;
        int next = targetLED + 1;
        if (prev >= LED_SKIP) {
          leds[prev] = CRGB(150, 60, 0);
        }
        if (next < NUM_LEDS) {
          leds[next] = CRGB(150, 60, 0);
        }
      }
      FastLED.show();
      
    } else if (mode == MODE_BALANCED) {
      FastLED.clear();
      for (int i = LED_SKIP; i < NUM_LEDS; i += 4) {
        leds[i] = CRGB(0, 255, 0);
      }
      FastLED.show();
    }
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize PWM driver
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  
  // Wait for oscillator to stabilize
  delay(10);
  
  // Initialize FastLED
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  
  // Initialize variables
  targetAngles[0] = NEUTRAL_ANGLE;
  targetAngles[1] = NEUTRAL_ANGLE;
  targetAngles[2] = NEUTRAL_ANGLE;
  
  // Optional: Send ready message
  Serial.println("Arduino servo controller ready (I2C PWM Driver + WS2812B LEDs). Echo off.");
}

void loop() {
  // Check for incoming serial data
  if (Serial.available() >= 3) {
    // Read first byte to determine command type
    int firstByte = Serial.read();
    
    if (firstByte == 250) {
      // LED command (250 = LED indicator, mode byte, data byte)
      int mode = Serial.read();
      int data = Serial.read();
      
      // Update LED mode
      if (mode >= 0 && mode <= 3) {
        setLEDMode((LEDMode)mode, data);
      }
      
    } else {
      // Servo command (3 bytes: angle0, angle1, angle2)
      int angle0 = firstByte;
      int angle1 = Serial.read();
      int angle2 = Serial.read();
      
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
  updateLEDAnimation();
}