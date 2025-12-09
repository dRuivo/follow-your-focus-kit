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


void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");

  // Initialize encoder pins
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  
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
}

void loop()
{
  // Read encoder position
  long encoderPos = getEncoderPosition();
  Serial.print("Encoder Position: ");
  Serial.println(encoderPos);
  
  int Pos;
  int Speed;
  int Load;
  int Voltage;
  int Temper;
  int Move;
  int Current;
  if(sms_sts.FeedBack(1)!=-1){
    Pos = sms_sts.ReadPos(-1);
    Speed = sms_sts.ReadSpeed(-1);
    Load = sms_sts.ReadLoad(-1);
    Voltage = sms_sts.ReadVoltage(-1);
    Temper = sms_sts.ReadTemper(-1);
    Move = sms_sts.ReadMove(-1);
    Current = sms_sts.ReadCurrent(-1);
    Serial.print("Position:");
    Serial.println(Pos);
    Serial.print("Speed:");
    Serial.println(Speed);
    Serial.print("Load:");
    Serial.println(Load);
    Serial.print("Voltage:");
    Serial.println(Voltage);
    Serial.print("Temper:");
    Serial.println(Temper);
    Serial.print("Move:");
    Serial.println(Move);
    Serial.print("Current:");
    Serial.println(Current);
    delay(10);
  }else{
    Serial.println("FeedBack err");
    delay(500);
  }
  
  Pos = sms_sts.ReadPos(1);
  if(Pos!=-1){
    Serial.print("Servo position:");
    Serial.println(Pos, DEC);
    delay(10);
  }else{
    Serial.println("read position err");
    delay(500);
  }
  
  Voltage = sms_sts.ReadVoltage(1);
  if(Voltage!=-1){
	Serial.print("Servo Voltage:");
    Serial.println(Voltage, DEC);
    delay(10);
  }else{
    Serial.println("read Voltage err");
    delay(500);
  }
  
  Temper = sms_sts.ReadTemper(1);
  if(Temper!=-1){
    Serial.print("Servo temperature:");
    Serial.println(Temper, DEC);
    delay(10);
  }else{
    Serial.println("read temperature err");
    delay(500);    
  }

  Speed = sms_sts.ReadSpeed(1);
  if(Speed!=-1){
    Serial.print("Servo Speed:");
    Serial.println(Speed, DEC);
    delay(10);
  }else{
    Serial.println("read Speed err");
    delay(500);    
  }
  
  Load = sms_sts.ReadLoad(1);
  if(Load!=-1){
    Serial.print("Servo Load:");
    Serial.println(Load, DEC);
    delay(10);
  }else{
    Serial.println("read Load err");
    delay(500);    
  }
  
  Current = sms_sts.ReadCurrent(1);
  if(Current!=-1){
    Serial.print("Servo Current:");
    Serial.println(Current, DEC);
    delay(10);
  }else{
    Serial.println("read Current err");
    delay(500);    
  }

  Move = sms_sts.ReadMove(1);
  if(Move!=-1){
    Serial.print("Servo Move:");
    Serial.println(Move, DEC);
    delay(10);
  }else{
    Serial.println("read Move err");
    delay(500);    
  }
  Serial.println();
  delay(1000);
}
