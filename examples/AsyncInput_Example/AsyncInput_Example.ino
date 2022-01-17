#include "asyncinput.h"

#define FLASH_BTN   0
#define BUTTON_2    34
#define ROTARY_BTN  35
#define PIR_SENSOR  36

#define ROTARY_A    32
#define ROTARY_B    33

uint16_t aButtonGpios[] = {
  FLASH_BTN | AsyncInput::LOWAKTIV,
  BUTTON_2,
  ROTARY_BTN | AsyncInput::LOWAKTIV | AsyncInput::PULL_UP,
  PIR_SENSOR
};

AsyncInput myinputs = AsyncInput(4, aButtonGpios);

void myBtn2Callback(int16_t repeats, uint16_t flags);
void myCallback1(int16_t repeats, uint16_t flags);
void myCallback2(int16_t repeats, uint16_t flags);
void myCallback3(int16_t repeats, uint16_t flags);
void myCallback4(int16_t repeats, uint16_t flags);
void myCallback5(int16_t rotaryDirection);

void setup() {
  delay(100);
  Serial.begin(115200);
  delay(100);
  Serial.printf("User async input example\n");

  myinputs.onButtonPress(myBtn2Callback, BUTTON_2, AsyncInput::REPEATING /*| AsyncInput::SIMULTAN*/);
  myinputs.onButtonRelease(myCallback1, BUTTON_2);
  myinputs.onButtonPress(myCallback2, ROTARY_BTN, AsyncInput::PRESS | AsyncInput::RELEASE);
  myinputs.onButtonSimultanPress(myCallback3, 2, BUTTON_2, ROTARY_BTN);
  myinputs.onButtonPress(myCallback4, PIR_SENSOR);

  myinputs.onRotaryEncoder(myCallback5, ROTARY_A | AsyncInput::PULL_UP, ROTARY_B | AsyncInput::PULL_UP);

  if (!myinputs.begin()) {
    Serial.println("ERROR : Task not created!");
  } else {
    Serial.println("Success!");
  }
}

void loop() {
  delay(1000);
  Serial.print(".");
}

// --- Callback functions ---

void myBtn2Callback(int16_t repeats, uint16_t flags) {
  Serial.printf("Button Press Callback : %d | %04x\n", repeats, flags);
}

void myCallback1(int16_t repeats, uint16_t flags) {
  Serial.printf("Button Release Callback : %d | %04x\n", repeats, flags);
}

void myCallback2(int16_t repeats, uint16_t flags) {
  if (repeats > 0) {
    Serial.printf("Rotary Press Callback : %d | %04x\n", repeats, flags);
  } else {
    Serial.printf("Rotary Release Callback : %d | %04x\n", repeats, flags);
  }
}

void myCallback3(int16_t repeats, uint16_t flags) {
  Serial.printf("Simultanous Press Callback : %d | %04x\n", repeats, flags);
}

void myCallback4(int16_t repeats, uint16_t flags) {
  Serial.printf("Motion Detected : %d | %04x\n", repeats, flags);
}

void myCallback5(int16_t rotaryDirection) {
  Serial.printf("Rotary Encoder : %d\n", rotaryDirection);
}