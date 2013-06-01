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

// EEPROM locations (only 512bytes!)
#define EEPROM_VOLTAGE  10
#define EEPROM_LIGHT    14
#define EEPROM_RELAY    18

// some states
#define PRESSED     1
#define NOT_PRESSED 0
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

#define BUTTON_DEBOUNCEtime 100 // ms

// support for sleep functionality
#include <avr/power.h>
#include <avr/sleep.h>
// other libraries
#include <EEPROM.h>
#include <TimerOne.h> // https://code.google.com/p/arduino-timerone/downloads/list

// global IR
// ** WARNING: be careful modifying these globals, as interrupt accesses them also
// ** WARNING: consuming too much RAM will cause arduino to fail to boot!
#define IRentries 128  // 128*3 = 768...close to 1Kbyte
#define TICKtime  10   // usec (100kHz)
char     IRdata[IRentries];
uint16_t IRtick[IRentries];
volatile int IRdataIndex = 0;
volatile int IRdataFull  = 0;

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
  pinMode(IRrx,        INPUT);
  pinMode(GREENled,    OUTPUT);
  pinMode(REDled,      OUTPUT);
  pinMode(RELAY,       OUTPUT);
  // extra stuff
  pinMode(LIGHTsensor, INPUT);
  pinMode(VOLTAGE,     INPUT);
  // unused stuff
  pinMode(IFportD2,    INPUT);
  pinMode(THENportD2,  OUTPUT);

  // initialize timer1 (used to demodulate IR from TSOP382)
  Timer1.initialize(TICKtime);
  Timer1.attachInterrupt(timer1_LEDdemodulateISR);

  // save initial state in EEPROM
  EEPROM.write(EEPROM_VOLTAGE, analogRead(VOLTAGE)/4);
  EEPROM.write(EEPROM_LIGHT, LIT);
  EEPROM.write(EEPROM_RELAY, RELAY_CLOSE);

  Serial.println("booted");
}

void loop()
{
  checkLightSensor();
  checkBattery();
  if (checkButton() == PRESSED)
  {
    toggleRelay();
    toggleStateLED();
  }
  updateActiveLED();

  if (IRdataFull)
    dumpIRdata();
}

void timer1_LEDdemodulateISR() // interrupt keyword unsupported in arduino?
{
  static uint16_t tick = 0; // wrap will occur!
  int LEDstate;
  uint16_t priorEntryTick;

  if (!IRdataFull)
  {
    LEDstate = digitalRead(IRrx);

    if (IRdataIndex == 0)
    {
      // always store first entry
      IRdata[IRdataIndex] = LEDstate;
      IRtick[IRdataIndex] = tick;
    }
    else if (LEDstate != IRdata[IRdataIndex - 1])
    {
      IRdata[IRdataIndex] = LEDstate;
      priorEntryTick = IRtick[IRdataIndex-1];
      if (priorEntryTick > tick)
        IRtick[IRdataIndex] = priorEntryTick + tick; // at least one wrap
      else
        IRtick[IRdataIndex] = tick - priorEntryTick;
    }
    IRdataIndex++;
    if (IRdataIndex == IRentries)
      IRdataFull = true;
  }
  tick++;
}

void dumpIRdata()
{
  Timer1.stop();

  Serial.println("dumpIRdata");
  Serial.println("index- state - time");
  for(IRdataIndex = 0; IRdataIndex < IRentries; IRdataIndex++)
  {
    Serial.print(IRdataIndex);
    Serial.print(" -  ");
    Serial.print(IRdata[IRdataIndex], DEC);
    Serial.print("  - ");
    Serial.println(IRtick[IRdataIndex]);
  }

  IRdataIndex = 0;

  Timer1.start();
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

int checkButton()
{
  static int lastPressTime = 0;
  int button = digitalRead(BUTTON);
  int state  = NOT_PRESSED;
  int now    = millis();

  if (button == LOW) // closed = pressed = pulled to GND
  {
    if (lastPressTime == 0)
      lastPressTime = now;
    else if ((now - lastPressTime) > BUTTON_DEBOUNCEtime)
    {
      DEBUG("button press");
      state = PRESSED;
      lastPressTime = 0;
    }
    // else pressed..but still bouncing, retain start of press
  }
  else lastPressTime = 0;

  return state;
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
