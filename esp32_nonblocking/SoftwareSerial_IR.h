/*
SoftwareSerial_IR.h - ESP32 Non-Blocking Port
Multi-instance software serial library for Arduino/Wiring
-- Based on the original SoftwareSerial_IR by curious
*/

#ifndef SoftwareSerial_IR_h
#define SoftwareSerial_IR_h

#include <inttypes.h>
#include <Stream.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

class SoftwareSerial_IR : public Stream
{
private:
  // per object data
  int8_t _transmitPin;
  uint16_t _tx_delay_ir;
  bool _inverse_logic;
  bool _disable_interrupts;

  QueueHandle_t _txQueue;
  TaskHandle_t _txTaskHandle;

  // private methods
  void setTX(int8_t transmitPin);
  static void txTask(void *pvParameters);
  void runTx();

public:
  // public methods
  SoftwareSerial_IR(int8_t transmitPin, bool inverse_logic = false, bool disable_interrupts = false);
  ~SoftwareSerial_IR();
  void begin(uint16_t speed);
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
