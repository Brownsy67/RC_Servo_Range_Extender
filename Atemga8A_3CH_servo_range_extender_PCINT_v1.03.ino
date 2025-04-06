#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>

// EEPROM layout: Address 0 = magic flag, then for each channel (3 channels) 6 bytes (MIN, MAX, MID)
#define CAL_MAGIC_ADDR 0
#define CAL_MAGIC 0xA5

// For ATmega8
#if defined(__AVR_ATmega8__)
  #define PCICR   GIMSK
  #define PCIE1   (1 << 6)
  #define PCMSK1  PCMSK
  #define PCINT8  0
  #define PCINT9  1
  #define PCINT10 2
  volatile uint8_t PCMSK;
#endif

const uint8_t pwmInputPins[3] = {A0, A1, A2};
const uint8_t pwmOutputPins[3] = {9, 10, 11};
const uint8_t buttonPin = 4, ledPin = 5;

Servo servos[3];
volatile uint16_t pulseStart[3] = {0}, pulseWidth[3] = {0};
volatile bool newPulse[3] = {false};

uint16_t pwmMin[3], pwmMax[3], pwmMid[3];
bool calibrating = false;
uint32_t lastPulseTime[3] = {0};

constexpr uint16_t defaultPwmMin = 988, defaultPwmMax = 2012, defaultPwmMid = 1500;

uint8_t currentChannel = 0;   // channel index: 0..2
uint8_t currentStep = 0;      // 0: MIN, 1: MAX, 2: MID
uint32_t lastCalibrationBlink = 0;
uint32_t calibrationLock = 0; // now 1 second lock

void loadCalibration() {
  if (EEPROM.read(CAL_MAGIC_ADDR) == CAL_MAGIC) {
    for (uint8_t i = 0; i < 3; i++) {
      EEPROM.get(1 + i * 6, pwmMin[i]);
      EEPROM.get(1 + i * 6 + 2, pwmMax[i]);
      EEPROM.get(1 + i * 6 + 4, pwmMid[i]);
    }
  } else {
    for (uint8_t i = 0; i < 3; i++) {
      pwmMin[i] = defaultPwmMin;
      pwmMax[i] = defaultPwmMax;
      pwmMid[i] = defaultPwmMid;
    }
  }
}

void saveCalibration() {
  for (uint8_t i = 0; i < 3; i++) {
    EEPROM.put(1 + i * 6, pwmMin[i]);
    EEPROM.put(1 + i * 6 + 2, pwmMax[i]);
    EEPROM.put(1 + i * 6 + 4, pwmMid[i]);
  }
  EEPROM.update(CAL_MAGIC_ADDR, CAL_MAGIC);
}

void quickBlink(uint8_t times) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}

void longBlink(uint8_t times) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}

void activeCalibrationBlink() {
  // Two rapid blinks every 1 second
  digitalWrite(ledPin, HIGH); delay(50);
  digitalWrite(ledPin, LOW); delay(50);
  digitalWrite(ledPin, HIGH); delay(50);
  digitalWrite(ledPin, LOW); delay(750);
}

void pwmISR() {
  for (uint8_t i = 0; i < 3; i++) {
    if (digitalRead(pwmInputPins[i]) == HIGH)
      pulseStart[i] = micros();
    else {
      pulseWidth[i] = micros() - pulseStart[i];
      newPulse[i] = true;
      lastPulseTime[i] = millis();
    }
  }
}

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(pwmInputPins[i], INPUT);
    servos[i].attach(pwmOutputPins[i], 500, 2500);
  }
  PCICR |= PCIE1;
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10);
  loadCalibration();
}

ISR(PCINT_vect) {
  pwmISR();
}

bool lastButtonState = HIGH;
void loop() {
  static uint32_t buttonHoldStart = 0;
  bool buttonState = digitalRead(buttonPin);
  
  // Enter calibration mode: hold button for 2 sec.
  if (!calibrating) {
    if (buttonState == LOW) {
      if (buttonHoldStart == 0)
        buttonHoldStart = millis();
      else if (millis() - buttonHoldStart > 2000) {
        calibrating = true;
        currentChannel = 0;
        currentStep = 0;
        lastCalibrationBlink = millis();
        calibrationLock = millis(); // now 1 sec lock
        buttonHoldStart = 0;
      }
    } else {
      buttonHoldStart = 0;
    }
  }
  
  if (calibrating) {
    activeCalibrationBlink(); // show 2 rapid blinks every second
    // Only 1 sec lock now:
    if (millis() - calibrationLock < 1000) {
      lastButtonState = buttonState;
      return;
    }
    // When button is pressed (transition from HIGH to LOW), record current value.
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
      quickBlink(3);  // Confirm with 3 quick blinks.
      currentStep++;
      calibrationLock = millis();  // 1 sec lock after press.
      // When channel complete, blink 5 times.
      if (currentStep > 2) {
        quickBlink(5);
        currentStep = 0;
        currentChannel++;
      }
      // When all channels calibrated, wait 1 sec then do final 3 long blinks.
      if (currentChannel >= 3) {
        delay(1000);
        longBlink(3);
        saveCalibration();
        calibrating = false;
      }
    }
    lastButtonState = buttonState;
  } else {
    // Normal operation: map pulse width to servo output.
    for (uint8_t i = 0; i < 3; i++) {
      if (newPulse[i]) {
        newPulse[i] = false;
        uint16_t constrained = constrain(pulseWidth[i], pwmMin[i], pwmMax[i]);
        uint16_t mapped = map(constrained, pwmMin[i], pwmMax[i], 500, 2500);
        servos[i].writeMicroseconds(mapped);
      }
    }
  }
}
