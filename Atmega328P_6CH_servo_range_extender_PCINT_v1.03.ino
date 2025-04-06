#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>

#define NUM_CHANNELS 6
#define CAL_MAGIC_ADDR 0
#define CAL_MAGIC 0xA5

const uint8_t pwmInputPins[NUM_CHANNELS] = {A0, A1, A2, A3, A4, A5};
const uint8_t pwmOutputPins[NUM_CHANNELS] = {3, 5, 6, 9, 10, 11};
const uint8_t buttonPin = 7;
const uint8_t ledPin = LED_BUILTIN;  // For ATmega328P, use LED_BUILTIN

Servo servos[NUM_CHANNELS];
volatile uint16_t pulseStart[NUM_CHANNELS] = {0}, pulseWidth[NUM_CHANNELS] = {0};
volatile bool newPulse[NUM_CHANNELS] = {false};

uint16_t pwmMin[NUM_CHANNELS], pwmMax[NUM_CHANNELS], pwmMid[NUM_CHANNELS];
bool calibrating = false;
uint32_t lastPulseTime[NUM_CHANNELS] = {0};

constexpr uint16_t defaultPwmMin = 988, defaultPwmMax = 2012, defaultPwmMid = 1500;

uint8_t currentChannel = 0;
uint8_t currentStep = 0;
uint32_t lastCalibrationBlink = 0;
uint32_t calibrationLock = 0;  // 1 sec lock now

void loadCalibration() {
  if (EEPROM.read(CAL_MAGIC_ADDR) == CAL_MAGIC) {
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
      EEPROM.get(1 + i * 6, pwmMin[i]);
      EEPROM.get(1 + i * 6 + 2, pwmMax[i]);
      EEPROM.get(1 + i * 6 + 4, pwmMid[i]);
    }
  } else {
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
      pwmMin[i] = defaultPwmMin;
      pwmMax[i] = defaultPwmMax;
      pwmMid[i] = defaultPwmMid;
    }
  }
}

void saveCalibration() {
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
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

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    pinMode(pwmInputPins[i], INPUT);
    servos[i].attach(pwmOutputPins[i], 500, 2500);
  }
  PCICR |= (1 << PCIE1);
  // Enable PCINT for A0-A5 (typically PCINT8..PCINT13)
  PCMSK1 |= (1UL << 8) | (1UL << 9) | (1UL << 10) | (1UL << 11) | (1UL << 12) | (1UL << 13);
  loadCalibration();
}

ISR(PCINT1_vect) {
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    if (digitalRead(pwmInputPins[i]) == HIGH)
      pulseStart[i] = micros();
    else {
      pulseWidth[i] = micros() - pulseStart[i];
      newPulse[i] = true;
      lastPulseTime[i] = millis();
    }
  }
}

bool lastButtonState = HIGH;
void loop() {
  static uint32_t buttonHoldStart = 0;
  bool buttonState = digitalRead(buttonPin);
  
  if (!calibrating) {
    if (buttonState == LOW) {
      if (buttonHoldStart == 0)
        buttonHoldStart = millis();
      else if (millis() - buttonHoldStart > 2000) {
        calibrating = true;
        currentChannel = 0;
        currentStep = 0;
        lastCalibrationBlink = millis();
        calibrationLock = millis();  // now 1 sec
        buttonHoldStart = 0;
      }
    } else {
      buttonHoldStart = 0;
    }
  }
  
  if (calibrating) {
    activeCalibrationBlink();
    if (millis() - calibrationLock < 1000) {  // 1 sec lock now
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
      if (currentChannel >= NUM_CHANNELS) { delay(1000); longBlink(3); saveCalibration(); calibrating = false; }
    }
    lastButtonState = buttonState;
  }
  else {
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
      if (newPulse[i]) {
        newPulse[i] = false;
        uint16_t constrained = constrain(pulseWidth[i], pwmMin[i], pwmMax[i]);
        uint16_t mapped = map(constrained, pwmMin[i], pwmMax[i], 500, 2500);
        servos[i].writeMicroseconds(mapped);
      }
    }
  }
}
