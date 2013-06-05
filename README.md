 A simple Arduino sketch which uses SeeedStudio's MilCandy and Grove hardware to
receive an IR code and toggle a relay.

 Usage:
  Press button on MilCandy to enable/disable learning (Red LED lit when learning)
  In learning mode, IR code from a remote is stored in EEPROM.
  In active mode, a matching IR code will toggle the relay

 ** Compilation requires https://github.com/shirriff/Arduino-IRremote library
 Arduino 1.5.2 environment was used from http://arduino.cc/en/Main/Software

Tested with hardware from SeeedStudio.com:
  MilCandy:          http://www.seeedstudio.com/wiki/MilCandy
  Grove IR receiver: http://www.seeedstudio.com/wiki/Grove_-_Infrared_Receiver
  Grove Relay:       http://www.seeedstudio.com/wiki/Grove_-_Relay
