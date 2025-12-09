#include <Arduino.h>
#include <SCServo.h>

SMS_STS sms_sts;

#define SERVO_RX_PIN 22
#define SERVO_TX_PIN 23
#define SERVO_ID 1

// Encoder pins
#define ENCODER_A_PIN 2   // Must be interrupt-capable pin
#define ENCODER_B_PIN 4   // Must be interrupt-capable pin

// Button pin
#define BUTTON_PIN 15

// Servo control parameters¨
const int MAX_SPEED = 2000; // Maximum servo speed
const float KP = 0.10; // Proportional gain for position control
const float KI = 0.02; // Integral gain for position control
const float KD = 2.0; // Derivative gain for position control
const float INTEGRAL_DECAY = 0.8; // Integral decay factor
const int DEADBAND = 4; // Position error deadband
const bool turningDirection = true; // true for one direction, false for the other

// Encoder variables
volatile long encoderPosition = 0;
volatile bool lastStateA = false;
volatile bool lastStateB = false;

// ISR for encoder channel A
void IRAM_ATTR encoderISR_A() {
  bool stateA = digitalRead(ENCODER_A_PIN);
  bool stateB = digitalRead(ENCODER_B_PIN);
  
  // Full quadrature decoding
  if (stateA != lastStateA) {
    if ((stateA == stateB) ^ turningDirection) {
      encoderPosition--;  // Counter-clockwise
    } else {
      encoderPosition++;  // Clockwise
    }
  }
  
  lastStateA = stateA;
}

// ISR for encoder channel B
void IRAM_ATTR encoderISR_B() {
  bool stateA = digitalRead(ENCODER_A_PIN);
  bool stateB = digitalRead(ENCODER_B_PIN);
  
  // Full quadrature decoding
  if (stateB != lastStateB) {
    if ((stateA == stateB) ^ turningDirection) {
      encoderPosition++;  // Clockwise
    } else {
      encoderPosition--;  // Counter-clockwise
    }
  }
  
  lastStateB = stateB;
}

// Function to safely read encoder position
long getEncoderPosition() {
  noInterrupts();
  long position = encoderPosition;
  interrupts();
  return position;
}

// Function to reset encoder position
void resetEncoderPosition() {
  noInterrupts();
  encoderPosition = 0;
  interrupts();
}

// Button debouncing variables
bool buttonPressed = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // milliseconds
// Gain settings
const float gainValues[] = {0.5, 1.0, 2.0, 4.0};
int currentGainIndex = 1; // Start with 1.0 gain
long lastEncoderPos = 0;

// Speed control variables
long currentServoPos = 0; // Current servo position
int lastServoPos = 0;
long targetPosition = 0; // Target position (can be infinite)

long last_error = 0;
long error_sum = 0;

// Function to calculate speed command based on position error
int calculateSpeedCommand(long targetPos, int currentPos) {
  // Calculate position error
  long error = targetPos - currentPos;
  
  // Apply deadband to avoid jitter
  if (abs(error) <= DEADBAND) {
    error_sum = 0;
    return 0; // Stop servo
  }
  error_sum = error_sum*INTEGRAL_DECAY + error; // Simple integral with decay
  
  // Calculate speed command using proportional control
  int speedCommand = (int)(error * KP) + (int)(error_sum * KI) + (int)((error - last_error) * KD);
  last_error = error;
  
  // Limit speed to maximum
  speedCommand = constrain(speedCommand, -MAX_SPEED, MAX_SPEED);
  
  return speedCommand;
}

// Function to handle button press for gain changes
void handleButtonPress() {
  bool reading = digitalRead(BUTTON_PIN);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonPressed) {
      buttonPressed = reading;
      
      if (buttonPressed == LOW) { // Button pressed (assuming active low)
        currentGainIndex = (currentGainIndex + 1) % (sizeof(gainValues) / sizeof(gainValues[0]));
        Serial.print("Gain changed to: ");
        Serial.println(gainValues[currentGainIndex]);
      }
    }
  }
  
  lastButtonState = reading;
}


void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");

  // Initialize encoder pins
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  
  // Initialize button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Read initial states
  lastStateA = digitalRead(ENCODER_A_PIN);
  lastStateB = digitalRead(ENCODER_B_PIN);
  
  // Attach interrupts for full quadrature decoding
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderISR_B, CHANGE);
  
  Serial.println("Encoder interrupts initialized");

   // Initialize the servo (only for tuning modes)
  Serial1.begin(1000000, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  sms_sts.pSerial = &Serial1; // Assign Serial1 to SCServo
  delay(1000);
  
  // Enable wheel mode for speed control
  if (sms_sts.WheelMode(SERVO_ID) == 0) {
    Serial.println("Servo set to wheel mode (speed control)");
  } else {
    Serial.println("Error: Failed to set wheel mode");
  }
  
  // Enable torque
  sms_sts.EnableTorque(SERVO_ID, 1);
  sms_sts.CalibrationOfs(SERVO_ID);
  delay(500);
  
  lastServoPos = sms_sts.ReadPos(SERVO_ID);

  Serial.print("Initial gain: ");
  Serial.println(gainValues[currentGainIndex]);
}

void loop()
{
  // Handle button press for gain change
  handleButtonPress();
  
  // Read encoder position
  long encoderPos = getEncoderPosition();
  
  // Calculate target position based on encoder and gain
  long encoderDelta = encoderPos - lastEncoderPos;
  targetPosition += (long)(encoderDelta * gainValues[currentGainIndex]);
  
  // Read current servo position
  int newServoPos = sms_sts.ReadPos(SERVO_ID);
  if (newServoPos != -1) {
    int deltaPos = newServoPos - lastServoPos;
    // Handle wrap-around
    if (deltaPos > 2048) {
      deltaPos -= 4096;
    } else if (deltaPos < -2048) {
      deltaPos += 4096;
    }
    currentServoPos += deltaPos;
    lastServoPos = newServoPos;
  }
  
  // Calculate speed command based on position error
  int speedCommand = calculateSpeedCommand(targetPosition, currentServoPos);
  
  // Send speed command to servo
  if (abs(encoderDelta) > 0 || abs(targetPosition - currentServoPos) > DEADBAND) {
    sms_sts.WriteSpe(SERVO_ID, speedCommand, 50); // Send speed command with acceleration
    lastEncoderPos = encoderPos;
  }
  
  // Display status information
  Serial.print("Encoder: ");
  Serial.print(encoderPos);
  Serial.print(", Gain: ");
  Serial.print(gainValues[currentGainIndex]);
  Serial.print(", Target: ");
  Serial.print(targetPosition);
  Serial.print(", Current: ");
  Serial.print(currentServoPos);
  Serial.print(", Speed Cmd: ");
  Serial.print(speedCommand);
  Serial.print(", Error: ");
  Serial.println(targetPosition - currentServoPos);
  
  delay(50); // Faster loop for better control response
}
