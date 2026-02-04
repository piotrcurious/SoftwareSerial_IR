/*
SoftwareSerial_IR.cpp - ESP32 Port
Multi-instance software serial library for Arduino/Wiring
-- Based on the original SoftwareSerial_IR by curious
*/

#include <Arduino.h>
#include "SoftwareSerial_IR.h"

//
// Constructor
//
SoftwareSerial_IR::SoftwareSerial_IR(int8_t transmitPin, bool inverse_logic, bool disable_interrupts) :
  _tx_delay_ir(0),
  _inverse_logic(inverse_logic),
  _disable_interrupts(disable_interrupts)
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
  // For IR mode, idle is LED OFF.
  // If inverse_logic is true (sink), LED OFF is HIGH.
  // If inverse_logic is false (source), LED OFF is LOW.
  digitalWrite(_transmitPin, _inverse_logic ? HIGH : LOW);
  pinMode(_transmitPin, OUTPUT);
}

//
// Public methods
//

void SoftwareSerial_IR::begin(uint16_t speed)
{
  // 1200 baud is 833.33 us per bit.
  // 36kHz is 27.77 us per cycle.
  // 833.33 / 27.77 = 30 cycles per bit.
  // _tx_delay_ir is the number of half-cycles, so 60.
  _tx_delay_ir = 60;
}

void SoftwareSerial_IR::end()
{
}

// Read data from buffer
int SoftwareSerial_IR::read()
{
    return -1; // not implemented
}

int SoftwareSerial_IR::available()
{
    return 0; // not implemented
}

size_t SoftwareSerial_IR::write(uint8_t b)
{
  int8_t pin = _transmitPin;
  uint8_t delay;
  bool pin_state = _inverse_logic ? HIGH : LOW; // Start with idle (LED OFF)

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
      // 1 is unmodulated (LED OFF)
      pin_state = _inverse_logic ? HIGH : LOW;
      digitalWrite(pin, pin_state);
      for (uint8_t j = 0; j < delay; j++) {
        delayMicroseconds(13);
      }
    }
    else {
      // 0 is modulated
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

  // Ensure pin stays in natural state
  digitalWrite(pin, _inverse_logic ? HIGH : LOW);

  if (_disable_interrupts) interrupts();
  return 1;
}

void SoftwareSerial_IR::flush() {}
int SoftwareSerial_IR::peek() { return -1; }
