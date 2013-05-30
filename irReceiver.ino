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
// Seems no bootloader on board? (no serial response & hooking PB5 LED doesn't blink)
// so non-bootloader programming is via:
//  PB3(MOSI),4(MISO),5(SCK),RST on board bottom as pads
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
//   3-  3- p01- PD3 = GreenLED
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

/////// ATmega168PV information
// page/chapter numbers from ATmega168PV Datasheet 8025M-AVR-6/11 - "doc8025")
// Timers:
//   Timer0  -  8bit (p92/ch15)  prescaler 0 <<< drives time for Arduino infrastructure
//   Timer1  - 16bit (p111/ch16) prescaler 0
//   Timer2  -  8bit (p142/ch18) allows asynchronous (eg externally driven can cause wakeup interrupt)
//   Watchdog - ch11.8/p51
//   SPI     - ch19/p164
//   USART   - ch20/p174 (supports a SPI mode)
//   I2C/TWI - ch22/p212 (2 wire interface for communication bus)
//   Analog  - ch23/p244 (Analog comparitor)
//   ADC     - ch24/p248 (Analog to Digital Converter)

// FYI around p271 has bootloader info including a simple bootloader copying ram to flash
// fortunately this is not required as MilCandy has a Arduino bootloader variant onboard
// NOTE: avrdude/arduino didn't want to download when using WinXP/UartSBee

// define hardware (use arduino pins as 'PDx' goes away in arduino 1.5.x)
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
#define RELAY_OPEN  0
#define RELAY_CLOSE 1

// activity LED (in this case two 0.5sec blinks with 5sec off)
#define ACTIVE_BLINKS    2
#define ACTIVE_BLINKtime 500 // msec
#define ACTIVE_OFFtime  5000 // msec (must be multiple of BLINKtime)
#define ACTIVE_LED GREENled

// support for sleep functionality
#include <avr/power.h>
#include <avr/sleep.h>
// EEPROM object
#include <EEPROM.h>

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
  }
  updateActiveLED();
}


void checkLightSensor()
{
  int light = digitalRead(LIGHTsensor);
  int lastLight = EEPROM.read(EEPROM_LIGHT);
  
  if (light != lastLight)
  {
    if (light == LIT)
      Serial.println("lighted");
    else
      Serial.println("not lit");
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
    Serial.print("voltage: ");
    Serial.println(battery);
    EEPROM.write(EEPROM_VOLTAGE, battery);
  }
}

int checkButton()
{
  int button = digitalRead(BUTTON);
  int button2;
  int state = NOT_PRESSED;

 reReadButton:
  if (button == 0) // closed = pressed = pulled to GND
  { // debounce
    delay(10);
    button2 = digitalRead(BUTTON);

    if ((button == button2) && (state == NOT_PRESSED))
    {
      Serial.println("button press");
      state = PRESSED;
      goto reReadButton;
    }
  }
  return state;
}      

void toggleRelay()
{
  int relay = EEPROM.read(EEPROM_RELAY);
  int state;

  if (relay == RELAY_CLOSE)
  {
    state = RELAY_OPEN;
    Serial.println("relay open");
  }
  else
  {
    state = RELAY_CLOSE;
    Serial.println("relay close");
  }

  digitalWrite(RELAY, state);
  EEPROM.write(EEPROM_RELAY, state);
}

// LED pattern as:
//..       ..     ..
#define ON  LOW
#define OFF HIGH
unsigned long lastActiveUpdate = 0;
#define DO_BLINK 1
#define OFF_TIME 2
int activeState = DO_BLINK;
int blinks = 0;
int activeLED = OFF;
void updateActiveLED()
{
  unsigned long now = millis();
  unsigned long delta = now - lastActiveUpdate;

  if (delta > ACTIVE_BLINKtime)
  {
    if ((activeState == OFF_TIME) &&
        (delta > ACTIVE_OFFtime))
    {
      lastActiveUpdate = now;
      activeState = DO_BLINK;
    }
    else if (activeState == DO_BLINK) // implied (delta > ACTIVE_BLINKtime))
    {
      lastActiveUpdate = now;
      if (activeLED == OFF)
      {
        digitalWrite(ACTIVE_LED, ON);
        activeLED = ON;
        blinks++;
      }
      else
      {
        digitalWrite(ACTIVE_LED, OFF);
        activeLED = OFF;
      }
      if (blinks >= ACTIVE_BLINKS)
      {
        blinks = 0;
        activeState = OFF_TIME;
      }
    }
  }      
}
