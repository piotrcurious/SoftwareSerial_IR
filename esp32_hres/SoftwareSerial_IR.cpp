/*
SoftwareSerial_IR.cpp - ESP32 High-Resolution Port
Multi-instance software serial library for Arduino/Wiring
-- Based on the original SoftwareSerial_IR by curious
*/

#include <Arduino.h>
#include "SoftwareSerial_IR.h"

//
// Constructor
//
SoftwareSerial_IR::SoftwareSerial_IR(int8_t transmitPin, bool inverse_logic, bool disable_interrupts) :
  _transmitPin(transmitPin),
  _inverse_logic(inverse_logic),
  _disable_interrupts(disable_interrupts),
  _txQueue(NULL),
  _txTaskHandle(NULL),
  _is_transmitting(false),
  _cycles_per_bit(0),
  _cycles_per_half_carrier(0)
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
  uint32_t f_cpu = getCpuFrequencyMhz() * 1000000;
  _cycles_per_bit = f_cpu / speed;
  _cycles_per_half_carrier = f_cpu / 72000; // 36kHz * 2

  if (_txQueue == NULL) {
    _txQueue = xQueueCreate(128, sizeof(uint8_t));
  }

  if (_txTaskHandle == NULL) {
    xTaskCreate(
      SoftwareSerial_IR::txTask,
      "IR_TX_HRes",
      2048,
      this,
      10, // High priority for precise timing
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

int SoftwareSerial_IR::read() { return -1; }
int SoftwareSerial_IR::available() { return 0; }

size_t SoftwareSerial_IR::write(uint8_t b)
{
  if (_txQueue == NULL) return 0;
  if (xQueueSend(_txQueue, &b, 0) == pdPASS) {
    return 1;
  }
  return 0;
}

void SoftwareSerial_IR::flush()
{
  while (_txQueue != NULL && (uxQueueMessagesWaiting(_txQueue) > 0 || _is_transmitting)) {
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
      _is_transmitting = true;
      int8_t pin = _transmitPin;
      bool idle_state = _inverse_logic ? HIGH : LOW;
      bool pin_state = idle_state;

      uint32_t start_bit_time;

      if (_disable_interrupts) noInterrupts();

      start_bit_time = esp_cpu_get_cycle_count();

      // Start bit (modulated)
      uint32_t bit_end = start_bit_time + _cycles_per_bit;
      uint32_t next_edge = esp_cpu_get_cycle_count() + _cycles_per_half_carrier;
      while ((int32_t)(bit_end - (uint32_t)esp_cpu_get_cycle_count()) > 0) {
        pin_state = !pin_state;
        digitalWrite(pin, pin_state);
        while ((int32_t)(next_edge - (uint32_t)esp_cpu_get_cycle_count()) > 0) ;
        next_edge += _cycles_per_half_carrier;
      }

      // 8 data bits
      for (uint8_t i = 0; i < 8; i++) {
        uint32_t next_bit_end = bit_end + _cycles_per_bit;
        if (b & 1) {
          // 1 is unmodulated (LED OFF)
          digitalWrite(pin, idle_state);
          while ((int32_t)(next_bit_end - (uint32_t)esp_cpu_get_cycle_count()) > 0) ;
        } else {
          // 0 is modulated
          next_edge = esp_cpu_get_cycle_count() + _cycles_per_half_carrier;
          while ((int32_t)(next_bit_end - (uint32_t)esp_cpu_get_cycle_count()) > 0) {
            pin_state = !pin_state;
            digitalWrite(pin, pin_state);
            while ((int32_t)(next_edge - (uint32_t)esp_cpu_get_cycle_count()) > 0) ;
            next_edge += _cycles_per_half_carrier;
          }
        }
        b >>= 1;
        bit_end = next_bit_end;
      }

      // Stop bits (unmodulated)
      digitalWrite(pin, idle_state);

      if (_disable_interrupts) interrupts();

      // Graceful multitasking: Use vTaskDelay for the stop bits
      // 2 stop bits at 1200 baud is ~1.66ms.
      // We can delay for 1ms and then yield if needed.
      vTaskDelay(1);

      _is_transmitting = false;
    }
  }
}

int SoftwareSerial_IR::peek() { return -1; }
