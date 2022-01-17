#include "asyncinput.h"

#define MAXBUTTONS							31    // maximale Anzahl an Tasten / GPIOs die abgerufen werden können
#define LOWESTGPIO							0
#define HIGHESTGPIO							39

#define BUTTONREQUESTINTERVAL		1		  // in Ticks (= ms)
#define BUTTONREPEATTIME				800   // Verzögerung für erste Wiederholung 
#define BUTTONCONINUESTIME			100   // Verzögerung für alle weiteren Wiederholungen
#define QUEUESIZE               32    // maximale Anzahl Ereignisse in der Queue

#define KEYSTROKECLUSTERSIZE    4     // Clustergröße zum Speichern
#define ROTARYCLUSTERSIZE       4     // Clustergröße zum Speichern

/******************************************************************************
  Funcion : Constructor
  Parameter :
    inputNumber - Number of Elements (GPIOs) in array
    gpios       - Array with GPIOs, low aktive GPIOs must invert i.e. ~32
  Return : -
  ToDo : -
  Comment :
******************************************************************************/
AsyncInput::AsyncInput(uint8_t _inputNumber, uint16_t _gpios[]) {
  uint8_t gpiospin;

  _repeatInterval = BUTTONREPEATTIME;
  _continueIntervall = BUTTONCONINUESTIME;

  hMutex = xSemaphoreCreateMutex();

  inputNumber = _inputNumber;
  if (inputNumber > MAXBUTTONS) {
    inputNumber = MAXBUTTONS;
  }

  pGpioStore = (uint8_t*)malloc(inputNumber);

  for (int i = 0; i < inputNumber; i++) {
    pGpioStore[i] = _gpios[i];
    gpiospin = pGpioStore[i] & 0x3f;

    if ((gpiospin >= LOWESTGPIO) && (gpiospin <= HIGHESTGPIO)) {
      // define pin as input with pullup
      if (_gpios[i] & PULL_UP) {
        pinMode(gpiospin, INPUT_PULLUP);
      } else if (_gpios[i] & PULL_DOWN) {
          pinMode(gpiospin, INPUT_PULLDOWN);
      } else {
        pinMode(gpiospin, INPUT);
      }
    }
  }
}

/******************************************************************************
  Funcion : Destructor
  Parameter : -
  Return : -
  ToDo : -
  Comment :
******************************************************************************/
AsyncInput::~AsyncInput() {
  // stop();
  free(_pKeyStrokeStore);
  free(_pRotaryEncoderStore);
  free(pGpioStore);
}

/******************************************************************************
  Funcion : Starts the work task to scan the gpios
  Parameter : -
  Return :
    false = a error has occurred, the task could not created
  ToDo : -
  Comment :
******************************************************************************/
bool AsyncInput::begin() {
  if (xTaskCreate(HID_Task, "Async_HID_Task", 2048, this, 1, NULL) != pdPASS) {
    // Task create not successfully
    return false;
  }

  return true;
}

/******************************************************************************
  Funcion : no function at the moment
  Parameter : -
  Return : -
  ToDo : -
  Comment :
******************************************************************************/
void AsyncInput::stop() {
}

/******************************************************************************
  Funcion : get the bit mask of a pin (only intern)
  Parameter :
    gpio -
  Return :
    the bit mask
  ToDo : -
  Comment :
******************************************************************************/
uint32_t AsyncInput::getButtonMask(uint8_t gpio) {
  uint32_t mask = 0;

  xSemaphoreTake(hMutex, portMAX_DELAY);
  for (int i = inputNumber; i >= 0; i--) {
    mask <<= 1;
    if ((pGpioStore[i] & 0x3f) == (gpio & 0x3f)) { // obersten Bits sind Flags
      mask |= 0b00000001;
    }
  }
  xSemaphoreGive(hMutex);
  return mask;
}

/******************************************************************************
  Funcion : allocating memory to store keystroke (callback function etc.)
  Parameter :
  Return :
    Pointer to new slot to store keystroke
  ToDo : -
  Comment :
******************************************************************************/
AsyncInput::sKeyStrokeSlot* AsyncInput::getKeyStrokeSlot() {
  sKeyStrokeSlot* pNewKeyStrokeSlot = NULL;

  xSemaphoreTake(hMutex, portMAX_DELAY);
  if (!(_KeyStrokeSlotNumber % KEYSTROKECLUSTERSIZE)) {
    if (!_KeyStrokeStoreSize) {
      // Es ist noch kein Speicher reserviert => neu zuweisen
      _KeyStrokeStoreSize += KEYSTROKECLUSTERSIZE;
      _pKeyStrokeStore = (sKeyStrokeSlot*)malloc(_KeyStrokeStoreSize * sizeof(sKeyStrokeSlot));
    } else {
      // Reservierter Speicher wird vergrößert
      _KeyStrokeStoreSize += KEYSTROKECLUSTERSIZE;
      _pKeyStrokeStore = (sKeyStrokeSlot*)realloc(_pKeyStrokeStore, _KeyStrokeStoreSize * sizeof(sKeyStrokeSlot));
    }
    if (_pKeyStrokeStore == NULL) {
      _KeyStrokeStoreSize = 0;
      xSemaphoreGive(hMutex);
      return NULL;
    }
  }

  pNewKeyStrokeSlot = &_pKeyStrokeStore[_KeyStrokeSlotNumber];
  _KeyStrokeSlotNumber++;

  xSemaphoreGive(hMutex);
  return pNewKeyStrokeSlot;
}

/******************************************************************************
  Funcion : Store the function and infos of a single keystroke callback
  Parameter :
    cbFunction - the function which is called when a button is pressed
    gpio - the number of a gpio, the gpio must first be listed in array in constructor
    flags - a combination of eInputEventType). Is empty the PRESS flag is set automatically
  Return :
    false - a error has occurred
  ToDo : -
  Comment :
******************************************************************************/
bool AsyncInput::onButtonPress(tButtonCallback cbFunction, int8_t gpio, uint16_t flags) {
  uint32_t uButtonMask;

  if (gpio < 0) {
    gpio = (~gpio) & 0x3f;
  }
  uButtonMask = getButtonMask(gpio);
  //Serial.printf("__GPIO Event : %d | %d | %08x\n", gpio, flags, uButtonMask);

  if (!uButtonMask) {
    return false;   // GPIO keiner Taste zugeordnet
  }

  sKeyStrokeSlot* pKeyStrokeSlot = getKeyStrokeSlot();
  if (pKeyStrokeSlot == NULL) {
    return false;   // Es kann kein Speicher für die Callback-Funktion zur Verfügung gestellt werden
  }

  xSemaphoreTake(hMutex, portMAX_DELAY);
  // Serial.printf("Fill Press Note : %x;   Mask : %08x;  Flags : %04x\n", pCallbackNote, uButtonMask, flags);
  pKeyStrokeSlot->uButtonMask = uButtonMask;
  pKeyStrokeSlot->cbFunction = cbFunction;
  pKeyStrokeSlot->repeatTimer = 0;
  pKeyStrokeSlot->repeatCounter = 0;
  if (!(flags & (PRESS | RELEASE))) {
    flags |= PRESS;
  }
  pKeyStrokeSlot->flags.all = flags;

  //Serial.printf("__GPIO Event : %d | %d | %08x\n", gpio, flags, uButtonMask);

  xSemaphoreGive(hMutex);
  return true;
}

/******************************************************************************
  Funcion : Store the function and infos of a button release callback
  Parameter :
    cbFunction - the function which is called when a button is pressed
    gpio - the number of a gpio, the gpio must first be listed in array in constructor
    flags - a combination of eInputEventType
  Return :
    false - a error has occurred
  ToDo : -
  Comment :
******************************************************************************/
bool AsyncInput::onButtonRelease(tButtonCallback cbFunction, int8_t gpio) {
  return onButtonPress(cbFunction, gpio, RELEASE);
}

/******************************************************************************
  Funcion : Store the callback function and infos of a simultaneously press of
    several buttons
  Parameter :
    cbFunction - the function which is called when a button is pressed
    gpioNumber - the number of the following gpios
    gpio - the number of a gpio, the gpio must first be listed in array in constructor
  Return :
    false - a error has occurred
  ToDo : -
  Comment :
******************************************************************************/
bool AsyncInput::onButtonSimultanPress(tButtonCallback cbFunction, int gpioNumber, ... ) {
  int8_t gpio;
  uint32_t uButtonMask = 0;
  va_list ArgumentList;

  //Serial.printf("__Multi : %d | ", gpioNumber);
  va_start(ArgumentList, gpioNumber);
  for (int i = 0; i < gpioNumber; i++) {
    gpio = va_arg(ArgumentList, int);
    if (gpio < 0) {
      gpio = (~gpio) & 0x3f;
    }
    uButtonMask |= getButtonMask(gpio);
    //Serial.printf("%d | ", gpio);
  }
  va_end(ArgumentList);

  //Serial.printf("%08x\n", uButtonMask);

  if (!uButtonMask) {
    return false;   // GPIO keiner Taste zugeordnet
  }

  sKeyStrokeSlot* pKeyStrokeSlot = getKeyStrokeSlot();
  if (pKeyStrokeSlot == NULL) {
    return false;   // Es kann kein Speicher für die Callback-Funktion zur Verfügung gestellt werden
  }

  //Serial.printf("Fill Multi Note : %x;   Mask : %08x\n", pCallbackNote, uButtonMask);

  xSemaphoreTake(hMutex, portMAX_DELAY);

  pKeyStrokeSlot->uButtonMask = uButtonMask;
  pKeyStrokeSlot->cbFunction = cbFunction;
  pKeyStrokeSlot->repeatTimer = 0;
  pKeyStrokeSlot->repeatCounter = 0;
  pKeyStrokeSlot->flags.all = PRESS | SIMULTAN;

  xSemaphoreGive(hMutex);

  //Serial.printf("Fill Multi Note : %x;   Mask : %08x\n", pCallbackNote, pCallbackNote->uButtonMask);
  return true;
}

/******************************************************************************
  Funcion : Set the speed of keystroke repeat events
  Parameter :
    repeat - is the time in ms from pressing the button to the first repeat event
    continuerepeat - is the time in ms for each further repeat event
  Return : -
  ToDo : -
  Comment :
******************************************************************************/
void AsyncInput::setButtonRepeatInterval(uint16_t repeat, uint16_t continuerepeat) {
  xSemaphoreTake(hMutex, portMAX_DELAY);
  _repeatInterval = repeat;
  _continueIntervall = continuerepeat;
  xSemaphoreGive(hMutex);
}

/******************************************************************************
  Funcion : allocating memory to store rotary encoder (callback function etc.)
  Parameter :
  Return :
    Pointer to new slot to store a rotary encoder
  ToDo : -
  Comment :
******************************************************************************/
AsyncInput::sRotaryNote* AsyncInput::getRotaryEncoderSlot() {
  sRotaryNote* pNewRotaryNote = NULL;

  xSemaphoreTake(hMutex, portMAX_DELAY);
  if (!(_RotaryEncoderNoteNumber % ROTARYCLUSTERSIZE)) {
    if (!_RotaryEncoderStoreSize) {
      // Es ist noch kein Speicher reserviert
      _RotaryEncoderStoreSize += ROTARYCLUSTERSIZE;
      _pRotaryEncoderStore = (sRotaryNote*)malloc(_RotaryEncoderStoreSize * sizeof(sRotaryNote));
    } else {
      // Reservierter Speicher wird vergrößert
      _RotaryEncoderStoreSize += ROTARYCLUSTERSIZE;
      _pRotaryEncoderStore = (sRotaryNote*)realloc(_pRotaryEncoderStore, _RotaryEncoderStoreSize * sizeof(sRotaryNote));
    }
    if (_pRotaryEncoderStore == NULL) {
      _RotaryEncoderStoreSize = 0;
      xSemaphoreGive(hMutex);
      return NULL;
    }
  }

  pNewRotaryNote = &_pRotaryEncoderStore[_RotaryEncoderNoteNumber];
  _RotaryEncoderNoteNumber++;

  xSemaphoreGive(hMutex);
  return pNewRotaryNote;
}

/******************************************************************************
  Funcion : Store the function and infos of a single keystroke callback
  Parameter :
    cbFunction - the function which is called when a button is pressed
    gpio_a - the gpio of one rotary encoder pin
    gpio_b - the gpio of the other rotary encoder pin
  Return :
    false - a error has occurred
  ToDo : -
  Comment :
******************************************************************************/
bool AsyncInput::onRotaryEncoder(tRotaryCallback cbFunction, uint16_t gpio_a, uint16_t gpio_b) {
  if ((gpio_a < LOWESTGPIO) || (gpio_a > HIGHESTGPIO) || (gpio_b < LOWESTGPIO) || (gpio_b > HIGHESTGPIO)) {
    //Serial.println("Ungültiger GPIO");
    return false;
  }

  if (cbFunction == NULL) {
    //Serial.println("Ungültige Callback funktion");
    return false;
  }

  sRotaryNote* pRotaryNote = getRotaryEncoderSlot();
  if (pRotaryNote == NULL) {
    //Serial.println("Kein Speicher");
    return false;   // Es kann kein Speicher reserviert werden
  }

  //Serial.printf("Rotary wird angelegt : %x | %d | %x\n", pRotaryNote, _RotaryEncoderNoteNumber, cbFunction);

  // define pin as input with pullup
  if (gpio_a & PULL_UP) {
    pinMode(gpio_a & 0x3f, INPUT_PULLUP);
  } else if (gpio_a & PULL_DOWN) {
    pinMode(gpio_a & 0x3f, INPUT_PULLDOWN);
  } else {
    pinMode(gpio_a & 0x3f, INPUT);
  }

  if (gpio_b & PULL_UP) {
    pinMode(gpio_b & 0x3f, INPUT_PULLUP);
  } else if (gpio_b & PULL_DOWN) {
    pinMode(gpio_b & 0x3f, INPUT_PULLDOWN);
  } else {
    pinMode(gpio_b & 0x3f, INPUT);
  }

  xSemaphoreTake(hMutex, portMAX_DELAY);
  //  pRotaryNote->id;
  pRotaryNote->GpioA = gpio_a;
  pRotaryNote->GpioB = gpio_b;
  pRotaryNote->cbFunction = cbFunction;
  pRotaryNote->stageCounter = 0;
  pRotaryNote->flag.lastRotaryStatusA = digitalRead(gpio_a);
  pRotaryNote->flag.lastRotaryStatusB = digitalRead(gpio_a);
  pRotaryNote->flag.rotarySnapLevel = pRotaryNote->flag.lastRotaryStatusA;
  xSemaphoreGive(hMutex);

  return true;
}

/******************************************************************************
  Funtion : Task Funktion zum abfragen und auswerten der GPIOs
  Parameter : -
  Rückgabe : -
  ToDo :
    -
  Bemerkung :
    Die Task-Funktion muss als "static" deklariert werden. Damit der Task
    auf die Klassen-Member zugreifen kann, muss dies über einen Pointer auf
    die Klasse geschehen. Der Pointer auf die Klasse wird meim erstellen
    des Tasks übergeben.
******************************************************************************/
void AsyncInput::HID_Task(void *pvParam) {
  AsyncInput *self = (AsyncInput*) pvParam;
  TickType_t lastWakeTime;

  uint32_t currentStates;
  uint32_t changedStates;
  uint32_t lastButtonStates = 0;
  //uint16_t repeatTimer = 0;
  //int16_t repeatsCounter = 0;
  //sInputEvent msg;

  //uint16_t repeatTimerAsync = 0;
  //int16_t repeatsCounterAsync = 0;
  uint16_t multiButtonCounter = 0;
  /*  union {
      uint16_t all;
      sButtonFlag flag;
    } tempButtonFlags;
  */
  uint16_t tempButtonFlags = 0;

  int16_t rotaryHelper;
  //int16_t rotaryCounter = 0;

  union {
    uint16_t all;
    struct {
      uint16_t gpioLevel: 1;
      uint16_t newEventOccurred: 1;
      //uint16_t repeatOccurred: 1;
      uint16_t simultaneouslyPress: 1;

      uint16_t rotaryStatusA: 1;
      uint16_t rotaryStatusB: 1;
    } flag;
  } flags;

  flags.all = 0;

  lastWakeTime = xTaskGetTickCount();
/*
  Serial.printf("__Task Started : %d\n", self->_KeyStrokeSlotNumber);

  for (int i = self->inputNumber-1; i >= 0; i--) {
    Serial.printf("__GPIOs :  %d | %04d | %04d | %04d\n", i, self->pGpioStore[i], self->pGpioStore[i] & 0x3f, self->pGpioStore[i] & LOWAKTIV);
  }

  for (int x = 0; x < self->_KeyStrokeSlotNumber; x++) {
    Serial.printf("__KeyStroke :  %d | %08x | %04x | %08x | %08x | %08x\n", x , self->_pKeyStrokeStore[x].uButtonMask, self->_pKeyStrokeStore[x].flags.all, self->_pKeyStrokeStore[x].cbFunction, self->_pKeyStrokeStore, &self->_pKeyStrokeStore[x]);
  }
  Serial.println("");
*/

  while (1) {
    xSemaphoreTake(self->hMutex, portMAX_DELAY);   // return pdFALSE oder pdTRUE

    // Alle aktuellen Stati der Tasten abfragen und zwischenspeichern
    multiButtonCounter = 0;
    currentStates = 0;
    for (int i = self->inputNumber-1; i >= 0; i--) {
      currentStates <<= 1;
      if (i >= LOWESTGPIO && i <= HIGHESTGPIO) {
        flags.flag.gpioLevel = digitalRead(self->pGpioStore[i] & 0x3f);
        if (self->pGpioStore[i] & LOWAKTIV) {
          flags.flag.gpioLevel ^= 1;
        }
        currentStates |= flags.flag.gpioLevel;
        multiButtonCounter += flags.flag.gpioLevel;
      }
    }
    // Ermitteln welche Stati sich geändert haben
    changedStates = currentStates ^ lastButtonStates;
    lastButtonStates = currentStates;

    //Serial.printf("__Inputs : %08x | %08x\n", currentStates, changedStates);

    if (multiButtonCounter > 1) {
      flags.flag.simultaneouslyPress = true;
    }

    for (int i = 0; i < self->_KeyStrokeSlotNumber; i++) {
      flags.flag.newEventOccurred = false;
      tempButtonFlags = 0;

      /*if (changedStates) {
        Serial.printf("__Changes : %d | %08x | %08x | %08x | %04x\n", i, currentStates, changedStates, self->_pKeyStrokeStore[i].uButtonMask, self->_pKeyStrokeStore[i].flags.all);
      }
      */

      if (changedStates & self->_pKeyStrokeStore[i].uButtonMask) {
        //if (((currentStates & self->_pKeyStrokeStore[i].uButtonMask) == self->_pKeyStrokeStore[i].uButtonMask) && self->_pKeyStrokeStore[i].flags.flag.Press) {
        if ((currentStates & self->_pKeyStrokeStore[i].uButtonMask) == self->_pKeyStrokeStore[i].uButtonMask) {
          self->_pKeyStrokeStore[i].repeatTimer = self->_repeatInterval;
          self->_pKeyStrokeStore[i].repeatCounter = 1;
          if (self->_pKeyStrokeStore[i].flags.flag.Press) {
            tempButtonFlags = PRESS;
            //Serial.printf("__Task + Button Press :  %d | %08x | %08x | %d\n", i, currentStates, self->_pKeyStrokeStore[i].uButtonMask, self->_pKeyStrokeStore[i].repeatCounter);
            flags.flag.newEventOccurred = true;
          }
        }
        if (!(currentStates & self->_pKeyStrokeStore[i].uButtonMask) && self->_pKeyStrokeStore[i].flags.flag.Release) {
          //Serial.printf("__Task - Button Release :  %d | %08x | %08x | %d\n", i, currentStates, self->_pKeyStrokeStore[i].uButtonMask, self->_pKeyStrokeStore[i].repeatCounter);
          self->_pKeyStrokeStore[i].repeatCounter = (~self->_pKeyStrokeStore[i].repeatCounter) + 1;
          tempButtonFlags = RELEASE;
          flags.flag.newEventOccurred = true;
        }
      } else {
        if ((currentStates & self->_pKeyStrokeStore[i].uButtonMask)/* && self->_pKeyStrokeStore[i].flags.flag.Repeat*/) {
          if (currentStates && !--self->_pKeyStrokeStore[i].repeatTimer) {
            self->_pKeyStrokeStore[i].repeatTimer = self->_continueIntervall;
            if (self->_pKeyStrokeStore[i].repeatCounter <= 32767) {
              self->_pKeyStrokeStore[i].repeatCounter++;
            }
            if (self->_pKeyStrokeStore[i].flags.flag.Repeat) { //(!flags.flag.simultaneouslyPress) { // ### evtl nicht notwendig
              //Serial.printf("__Task o Button Repeat :  %d | %08x | %08x | %d\n", i, currentStates, self->_pKeyStrokeStore[i].uButtonMask, self->_pKeyStrokeStore[i].repeatCounter);
              tempButtonFlags = REPEATING;
              flags.flag.newEventOccurred = true;
            }
          }
        }
      }
      if (flags.flag.newEventOccurred) {
        xSemaphoreGive(self->hMutex);
        if (flags.flag.simultaneouslyPress) {
          tempButtonFlags |= SIMULTAN;
        }
        self->_pKeyStrokeStore[i].cbFunction(self->_pKeyStrokeStore[i].repeatCounter, tempButtonFlags);
        xSemaphoreTake(self->hMutex, portMAX_DELAY);
        /*Serial.printf("__Call : \n");
          for (int j = 0; j < self->_KeyStrokeSlotNumber; j++) {
          Serial.printf("    %d | %08x | %08x | %08x\n", j , self->_pKeyStrokeStore[j].uButtonMask, self->_pKeyStrokeStore[j].cbFunction, self->_pKeyStrokeStore);
          }
          Serial.println("");
          */
        
      }
    }

    if (!currentStates) {
      flags.flag.simultaneouslyPress = false;
    }

    // Rotary Encoder
    for (int i = 0; i < self->_RotaryEncoderNoteNumber; i++) {
      flags.flag.rotaryStatusA = digitalRead(self->_pRotaryEncoderStore[i].GpioA);
      flags.flag.rotaryStatusB = digitalRead(self->_pRotaryEncoderStore[i].GpioB);

      rotaryHelper = 0;
      // Prüfen, auf welcher Leitung eine Flanke erkannt wurde
      if (self->_pRotaryEncoderStore[i].flag.lastRotaryStatusA ^ flags.flag.rotaryStatusA) {
        //Serial.printf("Flanke A, Enc No : %d\n", i);
        rotaryHelper = 2;
      }
      if (self->_pRotaryEncoderStore[i].flag.lastRotaryStatusB ^ flags.flag.rotaryStatusB) {
        //Serial.printf("Flanke B, Enc No : %d\n", i);
        rotaryHelper = 1;
      }

      // Prüfen, auf welcher Leitung zuerst eine Flanke eingetreten ist und Entprellung
      if (flags.flag.rotaryStatusA ^ flags.flag.rotaryStatusB) {
        self->_pRotaryEncoderStore[i].stageCounter += rotaryHelper;
      } else {
        self->_pRotaryEncoderStore[i].stageCounter -= rotaryHelper;
      }

      // Wenn ein komplette Raste weiter gedreht wurde, Ereignis auslösen und Fehler abfangen
      if (!(flags.flag.rotaryStatusA ^ flags.flag.rotaryStatusB)) {
        if (self->_pRotaryEncoderStore[i].flag.rotarySnapLevel != flags.flag.rotaryStatusA) {
          self->_pRotaryEncoderStore[i].flag.rotarySnapLevel = flags.flag.rotaryStatusA;
          if ((self->_pRotaryEncoderStore[i].stageCounter == -1) || (self->_pRotaryEncoderStore[i].stageCounter == 1)) {
            // Ereignis auslösen
            // Serial.printf("----- Rotary Enc : %01x | %01x | %d\n", flags.flag.rotaryStatusA, flags.flag.rotaryStatusB, rotaryCounter);
            // Serial.printf("Rotary Event : %x | %d\n", self->_pRotaryEncoderStore[i].cbFunction, self->_pRotaryEncoderStore[i].stageCounter);
            xSemaphoreGive(self->hMutex);
            self->_pRotaryEncoderStore[i].cbFunction(self->_pRotaryEncoderStore[i].stageCounter);
            xSemaphoreTake(self->hMutex, portMAX_DELAY);
          }
          self->_pRotaryEncoderStore[i].stageCounter = 0;
        }
      }

      // Status der Leitungen zwischenspeichern
      self->_pRotaryEncoderStore[i].flag.lastRotaryStatusA = flags.flag.rotaryStatusA;
      self->_pRotaryEncoderStore[i].flag.lastRotaryStatusB = flags.flag.rotaryStatusB;
    }

    xSemaphoreGive(self->hMutex);
    vTaskDelayUntil(&lastWakeTime, BUTTONREQUESTINTERVAL);
  }
  vTaskDelete(NULL);
}
