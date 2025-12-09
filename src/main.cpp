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
    if (stateA == stateB) {
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
    if (stateA == stateB) {
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
const int servoOffset = 2048; // Center position for 0-4095 range

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
        
        // Reset encoder position when changing gain to avoid jumps
        resetEncoderPosition();
        lastEncoderPos = 0;
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
  
  // Set initial servo position
  sms_sts.WritePosEx(SERVO_ID, servoOffset, 100, 50); // Move to center position
  delay(1000);
  
  Serial.print("Initial gain: ");
  Serial.println(gainValues[currentGainIndex]);
}

void loop()
{
  // Handle button press for gain change
  handleButtonPress();
  
  // Read encoder position
  long encoderPos = getEncoderPosition();
  
  // Calculate target servo position based on encoder and gain
  long encoderDelta = encoderPos - lastEncoderPos;
  int targetServoPos = servoOffset + (int)(encoderPos * gainValues[currentGainIndex]);
  
  // Constrain servo position to valid range (0-4095 for most servos)
  targetServoPos = constrain(targetServoPos, 0, 4095);
  
  // Only move servo if encoder position changed significantly
  if (abs(encoderDelta) > 0) {
    sms_sts.WritePosEx(SERVO_ID, targetServoPos, 500, 50); // Move servo with moderate speed
    lastEncoderPos = encoderPos;
  }
  
  // Display status information
  Serial.print("Encoder: ");
  Serial.print(encoderPos);
  Serial.print(", Gain: ");
  Serial.print(gainValues[currentGainIndex]);
  Serial.print(", Target Servo: ");
  Serial.println(targetServoPos);
  
  // Read current servo position for feedback (simplified)
  int currentServoPos = sms_sts.ReadPos(SERVO_ID);
  if(currentServoPos != -1) {
    Serial.print("Actual Servo Position: ");
    Serial.println(currentServoPos);
  }
  
  delay(50); // Small delay to avoid overwhelming the serial output
}
