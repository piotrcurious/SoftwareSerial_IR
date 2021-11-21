/*
SoftwareSerial.cpp (formerly NewSoftSerial.cpp) - 
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
-- bashing it all to make TX only, minimalist serial over IR library by Curious

IR serial axioms and changes :
-send with interrupts enabled because IR serial is slow enough . 
 You can pass interrupts_disable as parameter (after inverse_logic) to disable interrupts. 

-either delayMicroseconds used, or using equation to calculate cycle based delay. 
 equation stolen from :
https://raw.githubusercontent.com/NicoHood/IRLremote/master/extra/old/IRLremoteTransmit.hpp
 TODO: some choice? 
 WHY: delayMicroseconds should be aware of time spent in ISR and compensate for that. 
 at least in theory. Certainly possible on faster microcontrolers. 
 OTOH cycle based delay is more compact for tiny devices, but is more impacted by interrupts. 
 also on microcontrolers which allow dynamic changes of clock speed cycle based delay is not possible.
-removed all RX code, as it is not really anyhow beneficial to share code. 
-inverted logic stays as f.e. esp8266 has more sink capability than source. 
 it merely inverts IR modulation default (idle should be off) , it does not invert the RS232 logic.
 inverting RS232 logic makes no more sense as start bit must be 0 , and 0 means IR modulation. 
 it is not possible to generate start bit otherwise. 
-TODO: fixed baud rate of 1200baud. IR demodulators work best with this speed anyway. 
 1200baud means 30 IR pulses per bit, and 36khz means 27.777(...) uS per period.
-two stop bits. Transmission is not reliable without that as demodulator sometimes latches. 
 do not worry, most recievers do not care anyway. 
 perhaps it could be tweaked to 2400 or slowed down to 600baud, my experiments prove that it does
 not give much benefits though - slower rates are not really more reliable, and faster rate is 
 very tricky. perhaps with 56khz demodulator it could be better? 
-it is designed to add very nice display to tiny devices at really minimal cost 
 (single IR diode added) . no cables, no connectors, no galvanic connection. 
 this way device can be also hermetically enclosed (no holes for connectors) 
 it also consumes very little power, far less than any BT/wireless solution. 
 also it allows terminal device to be used for multiple devices. 
-also think about libraries like TVOut on slave devices and transmitting data to them in JSON 
 format - so dedicated 'display' boards, not just serial terminal protocol. 
 This allows much more room (f.e. using sprites, static text in progmem and many more features,
 like sounds) 
-data over IR solves problem of galvanic separation. 
 Easy to send data out from high voltage devices, cells of BMS systems,
 audio devices sensitive to ground loops etc. 
-TODO: TX buffer. As protocol is slow, some way to implement TX buffer would be great. 
 this is most difficult in my opinion, but it would free precious time on high cpu freq devices. 
 
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

// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <SoftwareSerial_IR.h>
#include <util/delay_basic.h>

//
// Private methods
//

/* static */ 
inline void SoftwareSerial_IR::tunedDelay(uint16_t delay) { 
  _delay_loop_2(delay);
}

//
// Constructor
//
SoftwareSerial_IR::SoftwareSerial_IR(int8_t transmitPin, bool inverse_logic,bool disable_interrupts /* = false */) : 
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
  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output high. Now, it is input with pullup for a short while, which
  // is fine. With inverse logic, either order is fine.
  digitalWrite(tx, _inverse_logic ? LOW : HIGH);
  pinMode(tx, OUTPUT);
  _transmitBitMask = digitalPinToBitMask(tx);
  uint8_t port = digitalPinToPort(tx);
  _transmitPortRegister = portOutputRegister(port);
}

//
// Public methods
//

void SoftwareSerial_IR::begin(uint16_t speed) // default 36khz, hardcoded FIXME  
{ 

	/*
	Bitbangs PWM in the given Hz number for the given time
	________________________________________________________________________________
	Delay calculation:
	F_CPU/1.000.000 to get number of cycles/uS
	/4 to get the number of loops needed for 1ms (1loop = 4 cycles)

	Multiply with the number of ms delay:
	1/kHz to get the seconds
	* 1.000.000 to get it in uS
	/2 to get half of a full pulse

	Substract the while, portmanipulation, loop overhead /4 loop cycles

	F_CPU(16.000.000)            1 * 1.000.000(pulse in uS)   12(overhead)
	========================== * ========================== - ==============
	1.000.000 * 4(loop cycles)   Hz * 2(half of a pulse)      4(loop cycles)

	<==>

	F_CPU(16.000.000) - (12(overhead) * Hz * 2(half of a pulse))
	===========================================================
	Hz * 2(half of a on/off pulse) * 4(loop cycles)

	________________________________________________________________________________
	Iterations calculation:
	Devide time with cycles in while loop
	Multiply this with the cycles per uS
	cycles per while loop: 4(loop cycles) * delay + overhead

	time * (F_CPU(16.000.000) / 1.000.000)
	======================================
	delay*4(loop cycles) + overhead
	*/
// TODO: pass the HZ here somehow
	const uint32_t IR_Hz = 36000 ;	// hardcoded FIXME
	const uint32_t loopCycles = 4;	// cycles in delay loop (1 call, 2 delay 1 return)
	const uint32_t overHead = 12; // just a guess from try + error
	const uint16_t IR_delay = (F_CPU - (overHead * IR_Hz * 2UL)) / (IR_Hz * 2UL * loopCycles);

  // Precalculate the various delays, in number of 4-cycle delays
//  uint16_t bit_delay = (F_CPU / speed) / 4;

  // Precalculate the various delays, in number of IR_delay delays
//  uint16_t IR_bit_delay = (F_CPU / speed) / (IR_delay+30);
//  uint16_t IR_bit_delay = 62;

  // 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
  // 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
  // 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
  // These are all close enough to just use 15 cycles, since the inter-bit
  // timings are the most critical (deviations stack 8 times)
  _tx_delay_ir = 60; // hardcoded. for 1200bps - 30 27.77 us cycles. so about 31 26us cycles. 
 //30*27.77=833.1us
 //32*26=832us 
 // 1200 baud should be 832uS 
 // so 31*26 = 806us plus some us of overheads. 

}

void SoftwareSerial_IR::end()
{
}


// Read data from buffer
int SoftwareSerial_IR::read()
{
    return -1; // not implemented so always report error. 
}

int SoftwareSerial_IR::available()
{
    return -1; // not implemented so always report error. 
}

size_t SoftwareSerial_IR::write(uint8_t b)
{

  // By declaring these as local variables, the compiler will put them
  // in registers _before_ disabling interrupts and entering the
  // critical timing sections below, which makes it a lot easier to
  // verify the cycle timings
  volatile uint8_t *reg = _transmitPortRegister;
  uint8_t reg_mask = _transmitBitMask;
  uint8_t inv_mask = ~_transmitBitMask;
  uint8_t oldSREG ; // define but not use. will be reduced if compiler manages to figure it out.  
if (_disable_interrupts)
  oldSREG = SREG; // optional, only if interrupts disabled 

  uint8_t delay = _tx_delay_ir; // in IR_DELAY cycles 
  uint16_t _IR_delay = IR_delay; // moved to local variable

//  if (inv)
//    b = ~b;  
// for IR mode inverse logic does not invert data, as it is not possible to send 
// serial over IR in inverted mode - IR demodulators active is 0, so only when modulated
// IR light is present, start bit will be generated. 
// inverse logic reverses IR modulation default (default is off when 0) 
// so it is possible to reverse it to use more sink capability of f.e. esp8266 . 
// so that is implemented. or rather the default stop bit is inversed, as for the rest
// of the IR modulation the bit is merely flipped. 

   if (_disable_interrupts)
	 cli();  // turn off interrupts for a clean txmit
// IR is slow enough to use delayMicroseconds.
// delayMicroseconds should compensate for time spent in ISR. 
// especially on fast microcontrolers. 
// on many arduino boards it does not, so if your trasmit fails, uncomment it back.
 
  // modulate IR signal
  while (delay--) {
  // flip pin state and wait for the calculated time
    *reg ^= reg_mask; // sent 1 is 0 - start bit. 
//     tunedDelay(_IR_delay);
	delayMicroseconds(13); //1/36000 = 27.77 microseconds
	}
  // Write each of the 8 bits
  for (uint8_t i = 8; i > 0; --i)
  {
  uint8_t delay = _tx_delay_ir; // moved to local variable, this increases overhead...
    if (b & 1) { // choose bit 
//modulate IR signal
		while (delay--){ // amount of IR delays
//      			*reg &= reg_mask; // 1 is unmodulated
			  if (_inverse_logic)
    				*reg |= reg_mask; // in sink , 1 is off
  			  else
    				*reg &= inv_mask; // IR mode - IR LED off. 

			delayMicroseconds(13); //1/36000 = 27.77 microseconds
//      		tunedDelay(_IR_delay); // IR delay
			}
    }
    else {
//send 0 
	while (delay--) { //amount of IR delays
      			*reg ^= reg_mask; // 0 is modulated
			delayMicroseconds(13); //1/36000 = 27.77 microseconds
//			tunedDelay(_IR_delay); // IR delay
			}
      }
    b >>= 1;
  }

  // restore pin to natural state (send stop bit)
  delay = _tx_delay_ir; // moved to local variable, this increases overhead...
  while (delay--) {
//	    	*reg |= reg_mask; // stop bit is 1 so do not modulate.  
  if (_inverse_logic)
    *reg |= reg_mask;
  else
    *reg &= inv_mask; // IR mode - IR LED off. 
		delayMicroseconds(27); //1/36000 = 27.77 microseconds
//     tunedDelay(_IR_delay);
		}
// stop bit twice

  delay = _tx_delay_ir; // moved to local variable, this increases overhead...
  while (delay--) {
//    		*reg |= reg_mask; // stop bit is 1 so do not modulate.  
  if (_inverse_logic)
    *reg |= reg_mask;
  else
    *reg &= inv_mask; // IR mode - IR LED off. 
		delayMicroseconds(14); //1/36000 = 27.77 microseconds
//     tunedDelay(_IR_delay);
	}
  // --------2nd stop bit 

//inverted for IR! 
  if (_inverse_logic)
    *reg |= reg_mask;
  else
    *reg &= inv_mask; // IR mode - IR LED off. 

if (_disable_interrupts)
  SREG = oldSREG; // turn interrupts back on only when interrupts were disabled
  return 1;
}

void SoftwareSerial_IR::flush()
{
  // There is no tx buffering, simply return
}

int SoftwareSerial_IR::peek()
{
    return -1;
}
