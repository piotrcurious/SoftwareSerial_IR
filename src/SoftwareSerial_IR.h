/*
SoftwareSerial_IR.h (formerly NewSoftSerial.h, stolen SoftwareSerial) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)
-- ATmega8/16/32/64/128/8515/8535 support by MCUdude (https://github.com/MCUdude)
-- Ugly bash to transmit IR modulated signal (roughly around 36khz...) by curious

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef SoftwareSerial_IR_h
#define SoftwareSerial_IR_h

#include <inttypes.h>
#include <Stream.h>


/******************************************************************************
* Definitions
******************************************************************************/

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

class SoftwareSerial_IR : public Stream
{
private:
  // per object data
  uint8_t _transmitBitMask;
  volatile uint8_t *_transmitPortRegister;

  // Expressed as 4-cycle delays (must never be 0!)
  uint16_t _tx_delay;
  uint16_t _tx_delay_ir;

// TODO: pass the HZ here somehow
        const uint32_t IR_Hz = 36000 ;  // hardcoded FIXME
        const uint32_t loopCycles = 4;  // cycles in delay loop (1 call, 2 delay 1 return)
        const uint32_t overHead = 15; // just a guess from try + error
        const uint16_t IR_delay = (F_CPU - (overHead * IR_Hz * 2UL)) / (IR_Hz * 2UL * loopCycles);

bool _inverse_logic;
bool _disable_interrupts; 

  // private methods
  void setTX(int8_t transmitPin);

  // private static method for timing
  static inline void tunedDelay(uint16_t delay);

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

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round

#endif
