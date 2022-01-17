/***********************************************************************
   Projekt : Tastenabfrage
   Author  : Martin.S
   Architecture : ESP32
  ---------------------------------------------------------------------
   Es sind unbedingt die Lizenzbestimmungen in der readme.txt zu
   beachten.
  ---------------------------------------------------------------------
   Historie:
     01.02.2020
  ---------------------------------------------------------------------
   ToDo :
     - Fehler abfrage (createqueue, createmutex)
     - Mutex erweitern
     - Task umstellen auf xTimerCreate oder xTaskDelayUntil
     - Wenn mehrere Tasten grückt wurden, danach einzellne ignorier.
  ---------------------------------------------------------------------
 *                                                                     *
   Copyright (c) 2020 Martin.S - martinsuniverse.de
 *                                                                     *
 ***********************************************************************/

#ifndef _ASYNC_INPUT_
#define _ASYNC_INPUT_

#include <Arduino.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define ASYNC_INPUT_VERSION_MAJOR   1
#define ASYNC_INPUT_VERSION_MINOR   0
#define ASYNC_INPUT_VERSION_PATCH   0
#define ASYNC_INPUT_VERSION_INFO    ""

class AsyncInput {
  public:
    enum eGpioType {
      LOWAKTIV    = 0x0080,
      PULL_UP     = 0x0100,
      PULL_DOWN   = 0x0200
    };

    enum eInputEventType {
      PRESS       = 0x0001,
      RELEASE     = 0x0002,
      REPEATING   = 0x0004,
      SIMULTAN    = 0x0008
    };

    typedef void (*tButtonCallback)(int16_t repeats, uint16_t flags);
    typedef void (*tRotaryCallback)(int16_t direction);

    AsyncInput(uint8_t _inputNumber, uint16_t _gpios[]);
    ~AsyncInput();
    bool begin();
    void stop();
    void setButtonRepeatInterval(uint16_t repeat, uint16_t continuerepeat);
    //bool isButtonInMask(uint8_t gpio);

    bool onButtonPress(tButtonCallback cbFunction, int8_t gpio, uint16_t flags = PRESS);
    bool onButtonRelease(tButtonCallback cbFunction, int8_t gpio);
    bool onButtonSimultanPress(tButtonCallback cbFunction, int gpioNumber, ... );
    //    void onButtonSimultanPress(tButtonCallback cbFunction, uint8_t gpioNumber, uint8_t gpios[]);

    bool onRotaryEncoder(tRotaryCallback cbFunction, uint16_t gpio_a, uint16_t gpio_b);

  private:
    typedef struct {
      //uint16_t id;    // ID-Level für mehrer Funktion der selben Taste z.B. in unterschiedlichen Fenstern
      uint32_t uButtonMask;
      tButtonCallback cbFunction;
      uint16_t repeatTimer;
      int16_t repeatCounter;
      union {
        uint16_t all;
        struct {
          uint16_t Press: 1;
          uint16_t Release: 1;
          uint16_t Repeat: 1;
          uint16_t Simultan: 1;
          uint16_t Invert: 1;
        } flag;
      } flags;
    } sKeyStrokeSlot;

    typedef struct {
      //  uint16_t id;
      uint8_t GpioA;
      uint8_t GpioB;
      tRotaryCallback cbFunction;
      int16_t stageCounter;
      struct {
        uint16_t lastRotaryStatusA: 1;
        uint16_t lastRotaryStatusB: 1;
        uint16_t rotarySnapLevel: 1;
      } flag;
    } sRotaryNote;

    //TaskHandle_t hHID_Task = NULL;
    SemaphoreHandle_t hMutex = NULL;

    uint8_t *pGpioStore = NULL;
    uint8_t  inputNumber;    // Anzahl der Registrierten GPIOs
    uint16_t _repeatInterval;
    uint16_t _continueIntervall;

    uint16_t _KeyStrokeStoreSize = 0;   // Größe des Reservierten Speichers in Anzahl Nodes
    uint16_t _KeyStrokeSlotNumber = 0;   // Anzahl an Notes
    sKeyStrokeSlot* _pKeyStrokeStore = NULL;
    sKeyStrokeSlot* getKeyStrokeSlot();   // Speicherverwaltung für Callbackstore

    uint16_t _RotaryEncoderStoreSize = 0;   // Größe des Reservierten Speichers in Anzahl Nodes
    uint16_t _RotaryEncoderNoteNumber = 0;   // Anzahl an Notes
    sRotaryNote* _pRotaryEncoderStore = NULL;
    sRotaryNote* getRotaryEncoderSlot();

    uint32_t getButtonMask(uint8_t gpio);
    static void HID_Task(void *pvParam);
};

#endif
