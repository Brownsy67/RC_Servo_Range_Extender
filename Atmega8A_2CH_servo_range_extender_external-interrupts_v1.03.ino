#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>

#define CAL_MAGIC_ADDR 0
#define CAL_MAGIC 0xA5

const uint8_t pwmInputPins[2] = {2, 3};
const uint8_t pwmOutputPins[2] = {9, 10};
const uint8_t buttonPin = 4, ledPin = 5;

Servo servos[2];
volatile uint16_t pulseStart[2] = {0}, pulseWidth[2] = {0};
volatile bool newPulse[2] = {false};

uint16_t pwmMin[2], pwmMax[2], pwmMid[2];
bool calibrating = false;
uint32_t lastPulseTime[2] = {0};

constexpr uint16_t defaultPwmMin = 988, defaultPwmMax = 2012, defaultPwmMid = 1500;

uint8_t currentChannel = 0;  // for 2 channels, use index 0
uint8_t currentStep = 0;
uint32_t lastCalibrationBlink = 0;
uint32_t calibrationLock = 0; // 1 sec lock

void loadCalibration() {
  if (EEPROM.read(CAL_MAGIC_ADDR) == CAL_MAGIC) {
    for (uint8_t i = 0; i < 2; i++) {
      EEPROM.get(1 + i * 6, pwmMin[i]);
      EEPROM.get(1 + i * 6 + 2, pwmMax[i]);
      EEPROM.get(1 + i * 6 + 4, pwmMid[i]);
    }
  } else {
    for (uint8_t i = 0; i < 2; i++) {
      pwmMin[i] = defaultPwmMin;
      pwmMax[i] = defaultPwmMax;
      pwmMid[i] = defaultPwmMid;
    }
  }
}

void saveCalibration() {
  for (uint8_t i = 0; i < 2; i++) {
    EEPROM.put(1 + i * 6, pwmMin[i]);
    EEPROM.put(1 + i * 6 + 2, pwmMax[i]);
    EEPROM.put(1 + i * 6 + 4, pwmMid[i]);
  }
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

void pwmISR(uint8_t i) {
  if (digitalRead(pwmInputPins[i]) == HIGH)
    pulseStart[i] = micros();
  else {
    pulseWidth[i] = micros() - pulseStart[i];
    newPulse[i] = true;
    lastPulseTime[i] = millis();
  }
}

void isr0() { pwmISR(0); }
void isr1() { pwmISR(1); }

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  for (uint8_t i = 0; i < 2; i++){
    pinMode(pwmInputPins[i], INPUT);
    servos[i].attach(pwmOutputPins[i], 500, 2500);
  }
  attachInterrupt(digitalPinToInterrupt(pwmInputPins[0]), isr0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwmInputPins[1]), isr1, CHANGE);
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
        currentChannel = 0;
        currentStep = 0;
        lastCalibrationBlink = millis();
        calibrationLock = millis(); // 1 sec lock now
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
      uint16_t value = pulseWidth[currentChannel];
      if (value == 0) {
        if (currentStep == 0) pwmMin[currentChannel] = defaultPwmMin;
        else if (currentStep == 1) pwmMax[currentChannel] = defaultPwmMax;
        else pwmMid[currentChannel] = defaultPwmMid;
      } else {
        if (currentStep == 0) pwmMin[currentChannel] = value;
        else if (currentStep == 1) pwmMax[currentChannel] = value;
        else pwmMid[currentChannel] = value;
      }
      quickBlink(3);
      currentStep++;
      calibrationLock = millis();
      if (currentStep > 2) { quickBlink(5); currentStep = 0; currentChannel++; }
      if (currentChannel >= 2) { delay(1000); longBlink(3); saveCalibration(); calibrating = false; }
    }
    lastButtonState = buttonState;
  }
  else {
    for (uint8_t i = 0; i < 2; i++){
      if (newPulse[i]) {
        newPulse[i] = false;
        uint16_t constrained = constrain(pulseWidth[i], pwmMin[i], pwmMax[i]);
        uint16_t mapped = map(constrained, pwmMin[i], pwmMax[i], 500, 2500);
        servos[i].writeMicroseconds(mapped);
      }
    }
  }
}
