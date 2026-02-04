/*
SoftwareSerial_IR.cpp - ESP32 Non-Blocking Port
Multi-instance software serial library for Arduino/Wiring
-- Based on the original SoftwareSerial_IR by curious
*/

#include <Arduino.h>
#include "SoftwareSerial_IR.h"

//
// Constructor
//
SoftwareSerial_IR::SoftwareSerial_IR(int8_t transmitPin, bool inverse_logic, bool disable_interrupts) :
  _tx_delay_ir(60),
  _inverse_logic(inverse_logic),
  _disable_interrupts(disable_interrupts),
  _txQueue(NULL),
  _txTaskHandle(NULL)
{
  setTX(transmitPin);
}

//
// Destructor
//
SoftwareSerial_IR::~SoftwareSerial_IR()
{
  end();
}

void SoftwareSerial_IR::setTX(int8_t tx)
{
  _transmitPin = tx;
  digitalWrite(_transmitPin, _inverse_logic ? HIGH : LOW);
  pinMode(_transmitPin, OUTPUT);
}

//
// Public methods
//

void SoftwareSerial_IR::begin(uint16_t speed)
{
  if (_txQueue == NULL) {
    _txQueue = xQueueCreate(64, sizeof(uint8_t));
  }

  if (_txTaskHandle == NULL) {
    xTaskCreate(
      SoftwareSerial_IR::txTask,
      "IR_TX_Task",
      2048,
      this,
      5, // Higher priority
      &_txTaskHandle
    );
  }
}

void SoftwareSerial_IR::end()
{
  if (_txTaskHandle != NULL) {
    vTaskDelete(_txTaskHandle);
    _txTaskHandle = NULL;
  }
  if (_txQueue != NULL) {
    vQueueDelete(_txQueue);
    _txQueue = NULL;
  }
}

// Read data from buffer
int SoftwareSerial_IR::read() { return -1; }
int SoftwareSerial_IR::available() { return 0; }

size_t SoftwareSerial_IR::write(uint8_t b)
{
  if (_txQueue == NULL) return 0;

  if (xQueueSend(_txQueue, &b, 0) == pdPASS) {
    return 1;
  }
  return 0; // Queue full
}

void SoftwareSerial_IR::flush()
{
  // Wait until queue is empty
  while (_txQueue != NULL && uxQueueMessagesWaiting(_txQueue) > 0) {
    vTaskDelay(1);
  }
}

void SoftwareSerial_IR::txTask(void *pvParameters)
{
  SoftwareSerial_IR *self = (SoftwareSerial_IR *)pvParameters;
  self->runTx();
}

void SoftwareSerial_IR::runTx()
{
  uint8_t b;
  while (true) {
    if (xQueueReceive(_txQueue, &b, portMAX_DELAY) == pdPASS) {
      int8_t pin = _transmitPin;
      uint8_t delay;
      bool pin_state = _inverse_logic ? HIGH : LOW;

      if (_disable_interrupts) noInterrupts();

      // Start bit (modulated)
      delay = _tx_delay_ir;
      for (uint8_t i = 0; i < delay; i++) {
        pin_state = !pin_state;
        digitalWrite(pin, pin_state);
        delayMicroseconds(13);
      }

      // 8 bits
      for (uint8_t i = 8; i > 0; --i)
      {
        delay = _tx_delay_ir;
        if (b & 1) {
          pin_state = _inverse_logic ? HIGH : LOW;
          digitalWrite(pin, pin_state);
          for (uint8_t j = 0; j < delay; j++) {
            delayMicroseconds(13);
          }
        }
        else {
          for (uint8_t j = 0; j < delay; j++) {
            pin_state = !pin_state;
            digitalWrite(pin, pin_state);
            delayMicroseconds(13);
          }
        }
        b >>= 1;
      }

      // Stop bit 1 (unmodulated)
      delay = _tx_delay_ir;
      pin_state = _inverse_logic ? HIGH : LOW;
      digitalWrite(pin, pin_state);
      for (uint8_t i = 0; i < delay; i++) {
        delayMicroseconds(27);
      }

      // Stop bit 2 (unmodulated)
      delay = _tx_delay_ir;
      digitalWrite(pin, pin_state);
      for (uint8_t i = 0; i < delay; i++) {
        delayMicroseconds(14);
      }

      digitalWrite(pin, _inverse_logic ? HIGH : LOW);

      if (_disable_interrupts) interrupts();
    }
  }
}

int SoftwareSerial_IR::peek() { return -1; }
