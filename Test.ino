#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include <NewPing.h> // Include the NewPing library for ultrasonic sensor

//MUST CHANGE

//MODIFY TO LINE UP OPEN GAUNTLET SERVO ANGLES
int S0_Open = 70;
int S1_Open = 83;
int S2_Open = 120;
int S3_Open = 120;
int S4_Open = 90;
int S5_Open = 95;
int S6_Open = 120;
int S9_Open = 120;
int S10_Open = 110; //This is the servo that locks the gauntlet together 
int S11_Open = 40;
int S12_Open = 90;
int S14_Open = 65;
int S15_Open = 90;

//MODIFY TO LINE UP CLOSED GAUNTLET SERVO ANGLES
int S0_Closed = 116; //Servo that hinges top left over your arm
int S1_Closed = 35; //Servo that hinges top right over your arm
int S2_Closed = 70; //Servo that moves top left armor plate into position (one closer to wrist)
int S3_Closed = 70; //Servo that moves top left armor plate into position (one closer to forearm)
int S4_Closed = 145; //Servo that moves top right armor plate into position (one closer to wrist)
int S5_Closed = 140; //Servo that moves top right armor plate into position (one closer to forearm)
int S6_Closed = 5; //Servo that moves the center laser armor plate into position (one closer to wrist)
int S9_Closed = 5; //Servo that moves the center laser armor plate into position (one closer to forearm)
int S10_Closed = 30; //Servo that locks the gauntlet together 
int S11_Closed = 105; //Servo that moves the piece over hand into position (Left Side)
int S12_Closed = 25; //Servo that moves the piece over hand into position (Right Side)
int S14_Closed = 110; //Servo that moves the higher Armor piece over hand into position (Left Side)
int S15_Closed = 45; //Servo that moves the higher Armor piece over hand into position (Right Side)

//OPTIONAL CHANGES

//SERVO MOVE SPEED (Higher number equals slower move speed)
int S0and1_Speed = 2; //Servos that hinge top parts over your arm
int S2and3_Speed = 1; //Servos that moves top left armor plate into position
int S4and5_Speed = 1; //Servo that moves top right armor plate into position
int S6and9_Speed = 1; //Servo that moves the center laser armor plate into position
int S10_Speed = 1; //Servo that locks the gauntlet together 
int S11and12_Speed = 1; //Servo that moves the piece over hand into position 
int S14and15_Speed = 1; //Servo that moves the higher Armor piece over hand into position

//HOW MUCH DELAY THERE IS AFTER PRESSING THE BUTTON BEFORE MOVEMENT INITIATES (milliseconds)
int GauntletOpenDelay = 500;  
int GauntletCloseDelay = 500;

//YOU DO NOT HAVE TO CHANGE ANYTHING BELOW

#define SERVO_0_PIN 0  // Add a new servo
#define SERVO_1_PIN 1  // Add a new servo
#define SERVO_2_PIN 2  // Add a new servo
#define SERVO_3_PIN 3  // Add a new servo
#define SERVO_4_PIN 4  // Add a new servo
#define SERVO_5_PIN 5  // Add a new servo
#define SERVO_6_PIN 6  // Add a new servo
#define SERVO_7_PIN 7  // Add a new servo
#define SERVO_8_PIN 8  // Add a new servo
#define SERVO_9_PIN 9  // Add a new servo
#define SERVO_10_PIN 10  // Add a new servo
#define SERVO_11_PIN 11  // Add a new servo
#define SERVO_12_PIN 12  // Add a new servo
#define SERVO_13_PIN 13  // Add a new servo
#define SERVO_14_PIN 14  // Add a new servo
#define SERVO_15_PIN 15  // Add a new servo
#define TACT_SWITCH_PIN 2 
#define NEOPIXEL_PIN 13 // D13 (pin 13 on Arduino Nano)
#define SERVO_COUNT 13   // Number of servos

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_NeoPixel strip = Adafruit_NeoPixel(7, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

int GauntletState = 0;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Define pins for the ultrasonic sensor
#define TRIGGER_PIN 2
#define ECHO_PIN 3

NewPing sonar(TRIGGER_PIN, ECHO_PIN); // Initialize the ultrasonic sensor

// Define a variable to store the NeoPixel color
uint32_t neoPixelColor = strip.Color(0, 0, 0); // Initially, NeoPixel is off

// Define arrays for servo angles
int servoAngles[SERVO_COUNT] = {S0_Open, S1_Open, S2_Open, S3_Open, S4_Open, S5_Open, S6_Open, S9_Open, S10_Open, S11_Open, S12_Open, S14_Open, S15_Open};
int servoTargetAngles[SERVO_COUNT] = {S0_Open, S1_Open, S2_Open, S3_Open, S4_Open, S5_Open, S6_Open, S9_Open, S10_Open, S11_Open, S12_Open, S14_Open, S15_Open};

// Function declarations
void moveServo(int servoNumber, int targetAngle, int speed, bool turnOffServos = true);
void moveServos(int servoNumber1, int servoNumber2, int targetAngle1, int targetAngle2, int speed, bool turnOffServos = true);

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  pwm.begin();
  pwm.setPWMFreq(60); // Set the PWM frequency to 60Hz
  pinMode(TACT_SWITCH_PIN, INPUT_PULLUP);
  strip.begin();      // Initialize the NeoPixel strip
  strip.show();       // Initialize all pixels to 'off'
  delay(1000);        // Initial delay for servo initialization
  Serial.println("Starting servo initialization...");
}

void loop() {
  // Read distance from the ultrasonic sensor
  int distance = sonar.ping_cm();

  if (distance < 2) { // If an object is detected within 2 cm
    // Trigger the servo movement based on GauntletState
    if (GauntletState == 0) { // OPEN GAUNTLET
      delay(GauntletOpenDelay);
      Serial.println("Moving servos 0 and 1 to new positions (GauntletState=1)...");
      GauntletState = 1;
      moveServos(6, 9, S6_Open, S9_Open, 2, true); // Move servos 6 and 7 to angles S6_open and S7_open then turn them off
      delay(1); // Delay
      moveServos(14, 15, S14_Open, S15_Open, 4, true);
      delay(1); // Delay
      moveServos(11, 12, S11_Open, S12_Open, 4, true); 
      delay(1); // Delay
      moveServos(4, 5, S4_Open, S5_Open, 4, true); 
      delay(1); // Delay
      moveServos(2, 3, S2_Open, S3_Open, 4, true); 
      delay(1); // Delay
      moveServo(10, S10_Open, 1, true); 
      delay(1); 
      moveServos(0, 1, S0_Open, S1_Open, 1, false); 
      delay(1); 
      fadeNeoPixelOff(strip.Color(255, 255, 200), 2000);    //fade off neopixel
    } else if (GauntletState == 1) { // CLOSED GAUNTLET
      Serial.println("Moving servos 0 and 1 to new positions (GauntletState=0)...");
      GauntletState = 0;
      delay(GauntletCloseDelay);
      moveServos(0, 1, S0_Closed, S1_Closed, 1, false); // Move servos 0 and 1 to angles 45 and 90 at speed S0n1_Speed, 
      delay(100); // Delay 
      moveServo(10, S10_Closed, false); // Move locking servo
      delay(1); 
      moveServos(2, 3, S2_Closed, S3_Closed, 4, true); 
      delay(1); // Delay
      moveServos(4, 5, S4_Closed, S5_Closed, 4, true); 
      delay(1); // Delay
      moveServos(11, 12, S11_Closed, S12_Closed, 4, true); 
      delay(1); // Delay
      moveServos(14, 15, S14_Closed, S15_Closed, 4, true); 
      delay(1); // Delay
      moveServos(6, 9, S6_Closed, S9_Closed, 2, true); 
      delay(1); // Delay
      flickerNeoPixelWarmWhite(); // Flicker NeoPixel with a warm white color
      delay(1);
      fadeNeoPixelOn(strip.Color(255, 255, 200), 2000); // Fade NeoPixel on to warm white
    }
  }
}

void moveServos(int servoNumber1, int servoNumber2, int targetAngle1, int targetAngle2, int speed, bool turnOffServos = true) {
  int currentAngle1 = servoAngles[servoNumber1];
  int currentAngle2 = servoAngles[servoNumber2];

  while (currentAngle1 != targetAngle1 || currentAngle2 != targetAngle2) {
    if (currentAngle1 < targetAngle1) {
      currentAngle1++;
    } else if (currentAngle1 > targetAngle1) {
      currentAngle1--;
    }

    if (currentAngle2 < targetAngle2) {
      currentAngle2++;
    } else if (currentAngle2 > targetAngle2) {
      currentAngle2--;
    }

    pwm.setPWM(servoNumber1, 0, angleToPulse(currentAngle1));
    pwm.setPWM(servoNumber2, 0, angleToPulse(currentAngle2));
    delay(speed); // Adjust the delay for speed control
  }

  servoAngles[servoNumber1] = currentAngle1;
  servoAngles[servoNumber2] = currentAngle2;

  // Turn off the servos if specified
  if (turnOffServos) {
    pwm.setPWM(servoNumber1, 0, 0);
    pwm.setPWM(servoNumber2, 0, 0);
  }
}

void moveServo(int servoNumber, int targetAngle, int speed, bool turnOffServos = true) {
  int currentAngle = servoAngles[servoNumber];

  while (currentAngle != targetAngle) {
    if (currentAngle < targetAngle) {
      currentAngle++;
    } else if (currentAngle > targetAngle) {
      currentAngle--;
    }

    pwm.setPWM(servoNumber, 0, angleToPulse(currentAngle));
    delay(speed); // Adjust the delay for speed control
  }

  servoAngles[servoNumber] = currentAngle;

  // Turn off the servo if specified
  if (turnOffServos) {
    pwm.setPWM(servoNumber, 0, 0);
  }
}

int angleToPulse(int angle) {
  // Convert degrees to PWM pulse width (approximate)
  return map(angle, 0, 180, 150, 600);
}

void fadeNeoPixelOn(uint32_t targetColor, int fadeDuration) {
  unsigned long startTime = millis();
  int steps = 100; // Number of steps for the fade (adjust as needed)
  int delayBetweenSteps = fadeDuration / steps;

  // Extract the red, green, and blue components from the targetColor
  int targetRed = (targetColor >> 16) & 0xFF;
  int targetGreen = (targetColor >> 8) & 0xFF;
  int targetBlue = targetColor & 0xFF;

  for (int i = 0; i <= steps; i++) {
    float brightness = map(i, 0, steps, 0, 255);
    uint32_t currentColor = strip.Color(
      targetRed * brightness / 255,
      targetGreen * brightness / 255,
      targetBlue * brightness / 255
    );

    for (int j = 0; j < 7; j++) {
      strip.setPixelColor(j, currentColor);
    }
    strip.show();
    delay(delayBetweenSteps);
  }
}

void fadeNeoPixelOff(uint32_t targetColor, int fadeDuration) {
  unsigned long startTime = millis();
  int steps = 100; // Number of steps for the fade (adjust as needed)
  int delayBetweenSteps = fadeDuration / steps;

  for (int i = steps; i >= 0; i--) {
    float brightness = map(i, 0, steps, 0, 255);
    uint32_t currentColor = strip.Color(
      (targetColor >> 16 & 0xFF) * brightness / 255,
      (targetColor >> 8 & 0xFF) * brightness / 255,
      (targetColor & 0xFF) * brightness / 255
    );

    for (int j = 0; j < 7; j++) {
      strip.setPixelColor(j, currentColor);
    }

    strip.show();
    delay(delayBetweenSteps);
  }
}

void flickerNeoPixelWarmWhite() {
  for (int i = 0; i < 1; i++) {
    // Randomly vary the brightness rapidly to create an aggressive flickering effect
    int flickerCount = random(10, 20); // Number of flicker steps
    neoPixelColor = strip.Color(255, 255, 200); // Warm white color

    for (int flickerStep = 0; flickerStep < flickerCount; flickerStep++) {
      int brightness = random(0, 150); // Random brightness between 0 and 255
      neoPixelColor = strip.Color(
        map(brightness, 0, 255, 0, 255),
        map(brightness, 0, 255, 0, 255),
        map(brightness, 0, 255, 0, 255)
      );

      for (int j = 0; j < 7; j++) {
        strip.setPixelColor(j, neoPixelColor);
      }

      strip.show(); // Show the updated pixel color
      delay(random(40, 100)); // Random short delay for flicker effect
    }

    // Delay for a slightly longer time between flicker cycles
    delay(random(200, 500)); // Random delay between flicker cycles
  }
}
