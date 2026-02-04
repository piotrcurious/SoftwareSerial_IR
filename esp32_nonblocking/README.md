# SoftwareSerial_IR (ESP32 Non-Blocking Port)
This is a non-blocking ESP32 port of the SoftwareSerial_IR library, using FreeRTOS tasks and queues.

IR serial axioms and changes :

* send with interrupts enabled because IR serial is slow enough .
 You can pass interrupts_disable as parameter (after inverse_logic) to disable interrupts.

* either delayMicroseconds used, or using equation to calculate cycle based delay.
 equation stolen from :
https://raw.githubusercontent.com/NicoHood/IRLremote/master/extra/old/IRLremoteTransmit.hpp
 TODO: some choice?
 WHY: delayMicroseconds should be aware of time spent in ISR and compensate for that.
 at least in theory. Certainly possible on faster microcontrolers.
 OTOH cycle based delay is more compact for tiny devices, but is more impacted by interrupts.
 also on microcontrolers which allow dynamic changes of clock speed cycle based delay is not possible.
* removed all RX code, as it is not really anyhow beneficial to share code.
* inverted logic stays as f.e. esp8266 has more sink capability than source.
 it merely inverts IR modulation default (idle should be off) , it does not invert the RS232 logic.
 inverting RS232 logic makes no more sense as start bit must be 0 , and 0 means IR modulation.
 it is not possible to generate start bit otherwise.
* TODO: fixed baud rate of 1200baud. IR demodulators work best with this speed anyway.
 1200baud means 30 IR pulses per bit, and 36khz means 27.777(...) uS per period.
* two long stop bits. Transmission is not reliable without that as demodulator sometimes latches.
 do not worry, most recievers do not care anyway.
 perhaps it could be tweaked to 2400 or slowed down to 600baud, my experiments prove that it does
 not give much benefits though - slower rates are not really more reliable, and faster rate is
 very tricky. perhaps with 56khz demodulator it could be better?
* it is designed to add very nice display to tiny devices at really minimal cost
 (single IR diode added) . no cables, no connectors, no galvanic connection.
 this way device can be also hermetically enclosed (no holes for connectors)
 it also consumes very little power, far less than any BT/wireless solution.
 also it allows terminal device to be used for multiple devices.
* also think about libraries like TVOut on slave devices and transmitting data to them in JSON
 format - so dedicated 'display' boards, not just serial terminal protocol.
 This allows much more room (f.e. using sprites, static text in progmem and many more features,
 like sounds)
* data over IR solves problem of galvanic separation.
 Easy to send data out from high voltage devices, cells of BMS systems,
 audio devices sensitive to ground loops etc.
* TODO: TX buffer. As protocol is slow, some way to implement TX buffer would be great.
 this is most difficult in my opinion, but it would free precious time on high cpu freq devices.
