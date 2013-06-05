// MilCandy IR receiver project
//  TSOP382 IR receiver connected to 'if'   port
//  NormallyOpen Relay  connected to 'then' port
// upon receiving an IR code, the relay will be toggled
//
// (C)opyright 10may2013 - Dale Schaafsma
//

/////// BOARD information...from various schematics/datasheets/code
// pins defined by MilCandy (Seeedstudio) - unconnected pins not identified
// VCC = 5V (based on calculations for DCtoDC transformer
//       Vout: (R1/R2+1)*Vfb=(4.5k/1.5k+1)*1.24=4.96V
// ATmega168PV (adds 6th sleep mode to ATmega168V, perhaps other enhancements)
// There is a variant of arduino bootloader: ATmegaBOOT_168_pro_8MHz.hex
// Program via arduino s/w or via PB3(MOSI),4(MISO),5(SCK),RST pads on board bottom
//
// pins are in: arduino- arduinoLogical- physical- datasheet
//        arduino tries to make it 'easier' by referring to arduino headers as 'pins'
//        see: hardware/arduino/avr/variants/standard/pins_arduino.h & arduino schematics
//  Port C can also map to ADC's for analog signals
//  16- A2- p25- PC2 = light sensor
//  17- A3- p26- PC3 = power detect (resistor network to battery)
//  18- A4- p27- PC4 = 'if' port I/O #1
//      ----->  data pin for TSOP382 IR receiver ('if'   port) (p2 on cable)
//  19- A5- p28- PC5 = 'if' port I/O #2
//
//  Port D
//   0-  0- p30- PD0 = rx on J3 (FTTDI header)
//   1-  1- p31- PD1 = tx on J3 (FTTDI header)
//   2-  2- p32- PD2 = button (no hardware debounce) normally open (reads VCC typically)
//   3-  3- p01- PD3 = GreenLED (if RedLed is lit, Green is NOT visible!)
//   4-  4- p02- PD4 = RedLED
//   5-  5- p09- PD5 = 'then' port I/O #1
//      ----->  data pin for Relay               ('then' port) (p1 on cable)
//   6-  6- p10- PD6 = 'then' port I/O #2
//   7-  7- p11- pd7 **NOTE: shows as LED in Seedstudio code, but is NC on board
//
//  FTDDI header (J3)
//    1 - DTR = RST  [closest to board center]
//    2 - PD1 = tx
//    3 - PD0 = rx
//    4 - VCC
//    5 - CTS = NC
//    6 - GND        [closest to board corner]
//
// Non-ATmega hardware information:
//   CN3083 charge controller: CH/OK tied to SMT red/green LED's
//   J5  = USB, only connected to VCC/GND
//   SW1 = power switch (no software access)
//   Grove cable pinout:
//      p1 - signal 1 (yellow)
//      p2 - signal 2 (white)
//      p3 - VCC      (red)
//      p4 - GND      (black)

/////// ATmega168PV information  (Flash:16kByte  EEPROM:512Byte  RAM:1kByte)
// page/chapter numbers from ATmega168PV Datasheet 8025M-AVR-6/11 - "doc8025")
// Timers:
//   Timer0   -  8bit (p92/ch15)  prescaler 0 <<< drives time for Arduino infrastructure
//   Timer1   - 16bit (p111/ch16) prescaler 0
//   Timer2   -  8bit (p142/ch18) allows asynchronous (eg externally driven can cause wakeup interrupt)
//   Watchdog - ch11.8/p51
//   SPI      - ch19/p164
//   USART    - ch20/p174 (supports a SPI mode)
//   I2C/TWI  - ch22/p212 (2 wire interface for communication bus)
//   Analog   - ch23/p244 (Analog comparitor)
//   ADC      - ch24/p248 (Analog to Digital Converter)
// FYI around p271 has bootloader info including a simple bootloader copying ram to flash
// fortunately this is not required as MilCandy has a Arduino bootloader variant onboard

/////// Arduino information
// **>> if size of RAM is exceeded, compile will work, but sketch fails to boot
//
// performance (from a technique at: http://jeelabs.org/2010/01/06/pin-io-performance/)
//  analogRead   ~10 samples/ms (resolution 0.1ms)
//  digitalRead ~200 samples/ms (resolution   5usec)
//  bitRead    ~1000 samples/ms (resolution   1usec)
//


// define hardware pins (use arduino pins as 'PDx' goes away in arduino 1.5.x)
#define RS232_RX    0   // PD0
#define RS232_TX    1   // PD1
#define BUTTON      2   // PD2 - input
#define GREENled    3   // PD3
#define REDled      4   // PD4
#define LIGHTsensor A2  // PC2 - input
#define VOLTAGE     A3  // PC3 - input
#define IFportD1    A4  // PC4
#define IFportD2    A5  // PC5
#define THENportD1  5   // PD5
#define THENportD2  6   // PD6

#define IRrx  IFportD1   // input
#define RELAY THENportD1 // output

// EEPROM locations (only 512bytes = 0x00 to 0x1ff!)
#define EEPROM_VOLTAGE    0x0a // 1 byte
#define EEPROM_LIGHT      0x0e // 1 byte
#define EEPROM_RELAY      0x10 // 1 byte
#define EEPROM_CODE_TYPE  0x20 // leave room for 4 bytes
#define EEPROM_CODE_VALUE 0x24 // 4 bytes

// some states
#define BUTTON_NOT_PRESSED HIGH  // open = pulled-up
#define BUTTON_PRESSED     LOW
#define BUTTON_CHANGE      2
#define BUTTON_NO_CHANGE   3
#define LIT         1
#define NOT_LIT     0
#define RELAY_OPEN  HIGH // normally open, active low
#define RELAY_CLOSE LOW
#define ON          HIGH
#define OFF         LOW

// state LED
#define STATE_LED  REDled
// activity LED (in this case two 0.5sec blinks with 5sec off)
#define ACTIVE_BLINKS    2
#define ACTIVE_BLINKtime 500 // msec (eg 0.5s on, 0.5s off)
#define ACTIVE_OFFtime  5000 // msec
#define ACTIVE_LED GREENled

// a good reference on debouncing: http://www.ganssle.com/debouncing.htm
#define BUTTON_DEBOUNCEtime 100 // ms - all samples must be same for valid state
#define BUTTON_DEBOUNCEperiod 5 // ms - each sample must be at least period apart

#define RELAY_MIN_TIME_BEFORE_CHANGE 1000 // ms

// support for sleep functionality (unused currently)
#include <avr/power.h>
#include <avr/sleep.h>
// other libraries
#include <EEPROM.h>
#include <IRremote.h>  // https://github.com/shirriff/Arduino-IRremote

// global IRremote objects
IRrecv irrecv(IRrx);

#define DO_DEBUG
#ifdef DO_DEBUG
#  define DEBUG(x)    Serial.println(x)
#  define DEBUG2(x,y) Serial.print(x);Serial.println(y)
#else
#  define DEBUG(x)
#  define DEBUG2(x,y)
#endif

void setup()
{
  Serial.begin(9600); // setup serial port well before output

  pinMode(BUTTON,      INPUT);
  irrecv.enableIRIn(); // sets IRrx as INPUT
  pinMode(GREENled,    OUTPUT);
  pinMode(REDled,      OUTPUT);
  pinMode(RELAY,       OUTPUT);
  // extra stuff
  pinMode(LIGHTsensor, INPUT);
  pinMode(VOLTAGE,     INPUT);
  // unused stuff
  pinMode(IFportD2,    INPUT);
  pinMode(THENportD2,  OUTPUT);

  // save initial state in EEPROM
  EEPROM.write(EEPROM_VOLTAGE, analogRead(VOLTAGE)/4);
  EEPROM.write(EEPROM_LIGHT, LIT);
  EEPROM.write(EEPROM_RELAY, RELAY_CLOSE);

  DEBUG("booted");
}

void loop()
{
  static unsigned long lastRelayToggle = millis();
  unsigned long now = millis();
  int storeIRcode;
  int buttonState;
  decode_results results;

  checkLightSensor();
  checkBattery();

  // only toggle STATE_LED on button changes for button press
  if ((checkButton(&buttonState) == BUTTON_CHANGE) &&
      (buttonState == BUTTON_PRESSED))
    toggleStateLED(); // returns LED on/off

  updateActiveLED();

  if (irrecv.decode(&results)) // only true when data available
  {
    dumpIRdata(&results);

    storeIRcode = digitalRead(STATE_LED);
    if (storeIRcode)
    {
      saveCodeToEEPROM(&results);
      digitalWrite(STATE_LED, OFF); // store first & only code! (next loop won't store)
      DEBUG("stored");
    }
    else if (isSavedCode(&results))
    {
      DEBUG("match!");
      // first MIN_TIME interval will disallow a toggle
      if ((now - lastRelayToggle) > RELAY_MIN_TIME_BEFORE_CHANGE)
      {
        toggleRelay();
        lastRelayToggle = now;
      }
    }

    irrecv.resume();     // clear everything and wait for next IR code
  }
}

void dumpIRdata(decode_results *results)
{
#ifdef DO_DEBUG
  switch(results->decode_type)
  {
  case NEC       : Serial.print("NEC"   ); break;
  case SONY      : Serial.print("SONY"  ); break;
  case RC5       : Serial.print("RC5"   ); break;
  case RC6       : Serial.print("RC6"   ); break;
  case DISH      : Serial.print("DISH"  ); break;
  case SHARP     : Serial.print("SHARP" ); break;
  case PANASONIC : Serial.print("PANAS: " ); Serial.print(results->panasonicAddress); break;
  case JVC       : Serial.print("JVC"   ); break;
  case SANYO     : Serial.print("SANYO" ); break;
  case MITSUBISHI: Serial.print("MITSUB"); break;
  case UNKNOWN   : Serial.print("UNK"   ); break;
  }
  Serial.print(" Val: ");
  Serial.println(results->value, HEX);
  Serial.print(" bit: ");
  Serial.println(results->bits);
  Serial.print(" rawlen: ");
  Serial.println(results->rawlen);
#endif
}

void saveCodeToEEPROM(decode_results *code)
{
  int loop;

  // truncate to char for efficiency
  EEPROM.write(EEPROM_CODE_TYPE, (code->decode_type & 0xff));

  // save all bytes of value MSB..LSB
  for(loop = 0; loop <= 3; loop++)
    EEPROM.write(EEPROM_CODE_VALUE + loop, ((code->value >> (8*loop)) & 0xff));
}

int isSavedCode(decode_results *code)
{
  int loop;
  int savedType = EEPROM.read(EEPROM_CODE_TYPE);
  unsigned long savedValue = 0;
  unsigned long byte;

  // read all bytes of value MSB..LSB
  for(loop = 3; loop >= 0; loop--)
  {
    byte = EEPROM.read(EEPROM_CODE_VALUE + loop);
    savedValue |= ( (byte & 0xff) << (8*loop) );
  }

  // minimal compare..but should be enough (truncate type to char for efficiency)
  return ( (savedType  == (code->decode_type & 0xff)) &&
           (savedValue ==  code->value) );
}

void checkLightSensor()
{
  int light = digitalRead(LIGHTsensor);
  int lastLight = EEPROM.read(EEPROM_LIGHT);
  
  if (light != lastLight)
  {
    if (light == LIT)
      DEBUG("lighted");
    else
      DEBUG("not lit");
    EEPROM.write(EEPROM_LIGHT, light);
  }
}

void checkBattery()
{
  int battery = analogRead(VOLTAGE)/4;
  int lastVoltage = EEPROM.read(EEPROM_VOLTAGE);

  if ((battery > (lastVoltage + 0.5)) ||
      (battery < (lastVoltage - 0.5)))
  {
    DEBUG2("voltage: ", battery);
    EEPROM.write(EEPROM_VOLTAGE, battery);
  }
}

int checkButton(int *returnState)
{
  static int steadyState = BUTTON_NOT_PRESSED;
  static int lastState   = BUTTON_NOT_PRESSED;
  static int pollCount   = BUTTON_DEBOUNCEtime / BUTTON_DEBOUNCEperiod;
  static unsigned long lastPollTime = 0;
  int button;
  int stateChanged  = BUTTON_NO_CHANGE;
  unsigned long now = millis();

  if (lastPollTime == 0)
    lastPollTime = now;  // init case, just return steadyState
  else if ((now - lastPollTime) > BUTTON_DEBOUNCEperiod)
  {
    // time to sample the button
    button = digitalRead(BUTTON);
    if (button == lastState)
    {
      pollCount--;
      if (pollCount == 0)
      {
        if (steadyState != lastState)
          stateChanged = BUTTON_CHANGE;

        // update debounced state & reset poll counter
        steadyState = lastState;
        pollCount   = BUTTON_DEBOUNCEtime / BUTTON_DEBOUNCEperiod;
        if (steadyState == BUTTON_PRESSED)
          DEBUG("button press");
      }
    }
    else
      lastState = button; // still bouncing, store last read
  }

  *returnState = steadyState;
  return stateChanged;
}

int toggleState(int currentState, int state1, int state2)
{
  if (currentState == state1)
    return state2;
  else if (currentState == state2)
    return state1;
  else
    return currentState;
}

void toggleRelay()
{
  int relay = EEPROM.read(EEPROM_RELAY);
  int state = toggleState(relay, RELAY_OPEN, RELAY_CLOSE);

  if (state == RELAY_OPEN)
    DEBUG("relay open");
  else
    DEBUG("relay close");

  digitalWrite(RELAY, state);
  EEPROM.write(EEPROM_RELAY, state);
}

void toggleStateLED()
{
  static int stateLED = OFF;
  int state = toggleState(stateLED, ON, OFF);
  digitalWrite(STATE_LED, state);
  stateLED = state;
}

// LED pattern as:
//..       ..     ..
void updateActiveLED()
{
  typedef enum {
    DO_BLINK = 1,
    OFF_TIME = 2
  } state_t;
  static unsigned long lastActiveUpdate = 0;
  static state_t activeState = DO_BLINK;
  static int blinks = 0;
  static int activeLED = OFF;
  unsigned long now = millis();
  unsigned long delta = now - lastActiveUpdate;

  switch(activeState)
  {
  case DO_BLINK:
    if (delta > ACTIVE_BLINKtime)
    {
      if (activeLED == OFF)
      {
        activeLED = ON;
        blinks++;
      }
      else // on, time to turn off
      {
        activeLED = OFF;
        if (blinks >= ACTIVE_BLINKS)
        {
          blinks = 0;
          activeState = OFF_TIME;
        }
      }
      digitalWrite(ACTIVE_LED, activeLED);
      lastActiveUpdate = now;
    }
    break;
  case OFF_TIME:
    if (delta > ACTIVE_OFFtime)
    {
      lastActiveUpdate = now;
      activeState = DO_BLINK;
    }
    break;
  default:
    break; // do nothing..illegal state
  }
}
