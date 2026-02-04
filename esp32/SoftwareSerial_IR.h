/*
SoftwareSerial_IR.h - ESP32 Port
Multi-instance software serial library for Arduino/Wiring
-- Based on the original SoftwareSerial_IR by curious
*/

#ifndef SoftwareSerial_IR_h
#define SoftwareSerial_IR_h

#include <inttypes.h>
#include <Stream.h>

class SoftwareSerial_IR : public Stream
{
private:
  // per object data
  int8_t _transmitPin;

  // Expressed as 4-cycle delays (must never be 0!)
  uint16_t _tx_delay_ir;

  bool _inverse_logic;
  bool _disable_interrupts;

  // private methods
  void setTX(int8_t transmitPin);

public:
  // public methods
  SoftwareSerial_IR(int8_t transmitPin, bool inverse_logic = false, bool disable_interrupts = false);
  ~SoftwareSerial_IR();
  void begin(uint16_t speed); // uint16_t because we only use 600/1200/2400
  void end();
  int peek();

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();
  virtual void flush();
  operator bool() { return true; }

  using Print::write;
};

#endif
