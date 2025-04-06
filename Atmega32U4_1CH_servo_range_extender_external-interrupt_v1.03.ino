#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>

#define CAL_MAGIC_ADDR 0
#define CAL_MAGIC 0xA5

const uint8_t pwmInputPin = 3;
const uint8_t pwmOutputPin = 9;
const uint8_t buttonPin = 4, ledPin = 30;

Servo myServo;
volatile uint32_t pulseStart = 0, pulseWidth = 0;
volatile bool newPulse = false;

uint16_t pwmMin, pwmMax, pwmMid;
bool calibrating = false;
uint32_t lastPulseTime = 0;

constexpr uint16_t defaultPwmMin = 988, defaultPwmMax = 2012, defaultPwmMid = 1500;

uint8_t currentStep = 0;
uint32_t lastCalibrationBlink = 0;
uint32_t calibrationLock = 0;  // 1 sec lock

void loadCalibration() {
  if (EEPROM.read(CAL_MAGIC_ADDR) == CAL_MAGIC) {
    EEPROM.get(1, pwmMin);
    EEPROM.get(3, pwmMax);
    EEPROM.get(5, pwmMid);
  } else {
    pwmMin = defaultPwmMin;
    pwmMax = defaultPwmMax;
    pwmMid = defaultPwmMid;
  }
}

void saveCalibration() {
  EEPROM.put(1, pwmMin);
  EEPROM.put(3, pwmMax);
  EEPROM.put(5, pwmMid);
  EEPROM.update(CAL_MAGIC_ADDR, CAL_MAGIC);
}

void activeCalibrationBlink() {
  digitalWrite(ledPin, HIGH); delay(50);
  digitalWrite(ledPin, LOW); delay(50);
  digitalWrite(ledPin, HIGH); delay(50);
  digitalWrite(ledPin, LOW); delay(750);
}

void quickBlink(uint8_t times) {
  for (uint8_t i = 0; i < times; i++){
    digitalWrite(ledPin, HIGH); delay(100);
    digitalWrite(ledPin, LOW); delay(100);
  }
}

void longBlink(uint8_t times) {
  for (uint8_t i = 0; i < times; i++){
    digitalWrite(ledPin, HIGH); delay(500);
    digitalWrite(ledPin, LOW); delay(500);
  }
}

void pwmISR() {
  if (digitalRead(pwmInputPin) == HIGH)
    pulseStart = micros();
  else {
    pulseWidth = micros() - pulseStart;
    newPulse = true;
    lastPulseTime = millis();
  }
}

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(pwmInputPin, INPUT);
  myServo.attach(pwmOutputPin, 500, 2500);
  attachInterrupt(digitalPinToInterrupt(pwmInputPin), pwmISR, CHANGE);
  loadCalibration();
}

bool lastButtonState = HIGH;
void loop() {
  static uint32_t buttonHoldStart = 0;
  bool buttonState = digitalRead(buttonPin);
  
  if (!calibrating) {
    if (buttonState == LOW) {
      if (buttonHoldStart == 0) buttonHoldStart = millis();
      else if (millis() - buttonHoldStart > 2000) {
        calibrating = true;
        currentStep = 0;
        lastCalibrationBlink = millis();
        calibrationLock = millis();
        buttonHoldStart = 0;
      }
    } else {
      buttonHoldStart = 0;
    }
  }
  
  if (calibrating) {
    activeCalibrationBlink();
    if (millis() - calibrationLock < 1000) { 
      lastButtonState = buttonState;
      return;
    }
    if (lastButtonState == HIGH && buttonState == LOW) {
      uint16_t value = pulseWidth;
      if (value == 0) {
        if (currentStep == 0) pwmMin = defaultPwmMin;
        else if (currentStep == 1) pwmMax = defaultPwmMax;
        else pwmMid = defaultPwmMid;
      } else {
        if (currentStep == 0) pwmMin = value;
        else if (currentStep == 1) pwmMax = value;
        else pwmMid = value;
      }
      quickBlink(3);
      currentStep++;
      calibrationLock = millis();
      if (currentStep > 2) { delay(1000); quickBlink(5); currentStep = 0; calibrating = false; saveCalibration(); }
    }
    lastButtonState = buttonState;
  }
  else {
    if (newPulse) {
      newPulse = false;
      uint16_t constrained = constrain(pulseWidth, pwmMin, pwmMax);
      uint16_t mapped = map(constrained, pwmMin, pwmMax, 500, 2500);
      myServo.writeMicroseconds(mapped);
    }
  }
}
