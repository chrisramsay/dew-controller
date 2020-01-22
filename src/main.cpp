// ==============================================================================================
// COPYRIGHT SECTION START
// Code for an astronomy 3-channel dew controller - automated
// (c) Copyright Robert Brown 2014-2019. All Rights Reserved.
// This project is protected by International Copyright Law.
// Permission is granted for personal and Academic/Educational use only.
// Software distributed under MIT License https://opensource.org/licenses/MIT

// PCB can be purchased online at https://go.aisler.net/p/HCXTNXFK

// CONTRIBUTIONS
// If you wish to make a small contribution in thanks for this project, please use PayPal and send the amount
// to user rbb1brown@gmail.com (Robert Brown). All contributions are gratefully accepted.

// ==============================================================================================
// NOTES: THIS CODE RUNS ON V3 BOARDS ONLY. USE WINDOWS APP v3028 or higher
// Serial port runs at 57600bps, Windows Application is coded to use 57600bps
// BT will connect to Windows App, ensure you pair the device and use virtual com port - see docs

// ==============================================================================================
#include "Settings.h"

// ==============================================================================================
// CONFIG SECTION START
// YOU MUST SET THESE DEFINES TO MATCH YOUR HARDWARE BEFORE COMPILING FIRMWARE
// This single file supports the DHT and HTU21D sensors, the LCD and OLED displays, and Bluetooth
// By default, the generated firmware is for LCD2004, DHT21 sensor, no bluetooth

// uncomment one of the following depending upon what sensor you are using
#define DHTXX    1
//#define HTU21DXX 2

// uncomment the next line if you want an LCDDISPLAY
#define LCDDISPLAY 1

// uncomment the next line if you want an OLED Display - but you cannot have both display types!!!!!!
//#define OLEDDISPLAY 1

// uncomment the next line if you want BLUETOOTH
#define BLUETOOTH 1

// only uncomment one of the following depending upon your lcd type
//#define LCD1602  1                    // 16 character, 2 lines
//#define LCD1604  2                    // 16 character, 4 lines
#define LCD2004  3                      // 20 character, 4 lines

#ifdef DHTXX                            // defines for each sensor type that is supported
// you must uncomment only ONE of the following lines to match the DHT sensor you are using
// if using a HTU21D sensor then leave MYSENSOR set to DHT22
//#define MYSENSOR DHT11
//#define MYSENSOR DHT21
#define MYSENSOR DHT22
//#define MYSENSOR DHT33
#endif

// ==============================================================================================
// DO NOT CHANGE ANYTHING IN THIS SECTION
#ifndef DHTXX
#ifndef HTU21DXX
#halt //Error - you do not have a sensor type defined - define either DHTXX or HTU21D
#endif
#endif

#ifdef DHTXX
#ifndef MYSENSOR
#halt //Error - you must set MYSENSOR to either DHT11, DHT21, DHT22 or DHT33 (even if you are using a HTU21D sensor
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifdef OLEDDISPLAY
#halt //Error - you can only have one LCD type defined - either LCDDISPLAY or OLEDDISPLAY.
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifdef LCD1602
#ifdef LCD1604
#halt //Error - you can only have one LCD type defined, one of LCD1602, LCD1604 or LCD2004.
#endif
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifdef LCD1602
#ifdef LCD2004
#halt //Error - you can only have one LCD type defined, one of LCD1602, LCD1604 or LCD2004.
#endif
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifdef LCD1604
#ifdef LCD2004
#halt //Error - you can only have one LCD type defined, one of LCD1602, LCD1604 or LCD2004.
#endif
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifdef LCD1602
#ifdef LCD2004
#ifdef LCD1604
#halt //Error - you can only have one LCD type defined, one of LCD1602, LCD1604 or LCD2004.
#endif
#endif
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifndef LCD2004
#ifndef LCD1604
#ifndef LCD1602
#halt //Error - you can must define either LCD1602 or LCD2004 or LCD1604.
#endif
#endif
#endif
#endif

// do not change
//#define DEBUG 1

// ==============================================================================================
// CHANGE REVISION SECTION START

// 3.33
// Implement settings file

// 3.32 30092019
// Add get displaymode so can set windows app to correct C or F on connect

// 3.31 02032019
// Add temperature setting at which pcb fan will switch off when under temperature control
// Sketch uses 28,788 bytes (93%)
// Global variables use 1,227 bytes (59%)

// 3.30 31072018
// Add get tracking mode offset value for use with Linux application

// 3.29 30032018
// Fix error in bluetooth

// 3.27 15032018
// Fix display bug in PCB temp and fan temp

// 3.26 05032018
// Fix display bug in humdity for OLED

// 3.25 11022018
// Add LCD1604 as display option, rewrite some LCD screens, add PCBTemp etc

// 3.24 08022018
// Fig bug in humidity display

// 3.22 27012018
// Rewrite using defines, all files and versions moved into single code stream
// Rewrite serial comms, implement queue

// ==============================================================================================
// FIRMWARE CODE START
// DO NOT CHANGE ANYTHING IN THIS SECTION

#include <Arduino.h>
#include <myQueue.h>                              //  By Steven de Salas
#include <Wire.h>                                 // needed for I2C
#include <math.h>
#ifdef DHTXX
#include <mydht.h>                                // needed for DHT11/21/22/33
#endif
#ifdef HTU21DXX
#include <mySparkFunHTU21D.h>                     // needed for HTU21D sensor
#endif
#include <OneWire.h>                              // needed for DS18B20 temperature probe
#include <myDallasTemperature.h>                  // needed for DS18B20 temperature probe
#include <myEEPROM.h>                             // needed for EEPROM v2.46 or higher
#include <myeepromanything.h>                     // needed for EEPROM v2.46 or higher
#ifdef BLUETOOTH
#include <SoftwareSerial.h>                       // needed for BT adapter
#endif
#ifdef LCDDISPLAY
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#endif
#ifdef OLEDDISPLAY
#include <mySSD1306Ascii.h>                       // oled
#include <mySSD1306AsciiWire.h>                   // oled
#endif

// ==============================================================================================
// GLOBAL VARS - DO NOT CHANGE ANYTHING IN THIS SECTION
char ver[] = "333";                               // do not change

Queue<String> queue(10);                          // receive serial queue of commands
char line[MAXCOMMAND];
int eoc;                                          // end of command
int idx;                                          // index into command string
int boardtemp;
OneWire oneWirech1(CH1TEMP);                      // setup temperature probe 1
OneWire oneWirech2(CH2TEMP);                      // setup temperature probe 2
OneWire oneWirech3(CH3TEMP);                      // setup temperature probe 3
OneWire oneWirefan(FANTEMP);                      // setup temperature for board - fan control
DallasTemperature sensor1(&oneWirech1);           // probe ch1
DallasTemperature sensor2(&oneWirech2);           // probe ch2
DallasTemperature sensor3(&oneWirech3);           // probe ch3
DallasTemperature sensor4(&oneWirefan);           // probe fan
DeviceAddress tpAddress;                          // used to send precision to specific sensor
int tprobe1;                                      // these indicate if there is a probe attached to that channel
int tprobe2;
int tprobe3;
int tprobe4;
bool displayenabled;                              // used to enable and disable the display
long buttonLastChecked;                           // variable to limit the button getting checked every cycle
int PBVal;                                        // holds state of toggle switch read
float ch1tempval;                                 // temperature value for each probe
float ch2tempval;
float ch3tempval;
float ch1oldtempval;                              // saved temp values for ch1/2
float ch2oldtempval;
float ch3oldtempval;
int ch1pwrval, ch2pwrval, ch3pwrval;              // percentage power to each channel
int ch3manualpwrval;                              // used to remember the manualpwrsetting for ch3
int hval;                                         // relative humidity
float tval;                                       // ambient temperature value
float dew_point;                                  // dewpoint
int ch1override, ch2override;                     // used in remote mode to override channels to 100%
int computeroverride;                             // 1 if computer is overriding to 100% power for ch1 or ch2
float TempF;                                      // used to hold conversion of temperatures to Fahrenheit
long pos;                                         // holds any parameter sent with command
long buttonchecktime;                             // time between switch override checks
int currentaddr;                                  // will be address in eeprom of the data stored
bool writenow;                                    // should we update values in eeprom
long currenttime;                                 // current timestamp
long displaytimer;                                // time of last display update
long temptimer;                                   // time of last temperature update
bool temprefresh;
bool pcbfanon;
String hash = "#";
String endofstr = "$";

// ==============================================================================================
// EEPROM DATA STRUCT - DO NOT CHANGE ANYTHING IN THIS SECTION
struct config_t {
  int validdata;
  int TrackingState;                    // algorithm to use is dew point temperature tracking
  int offsetval;                        // used to alter temperature boundaries in algorithm for determining when to apply power to dewstraps
  int fanspeed;                         // speed of cooling fan
  int fantempon;                        // the temperature at which the fan which be switched on, 0 is OFF
  int ATBias;                           // ambient temperature bias offset used to calibrate ambient temperature from dhtxx sensor
  float ch1offset;                      // channel1 temperature probe calibration offset value
  float ch2offset;                      // channel2 temperature probe calibration offset value
  float ch3offset;                      // channel3 temperature probe calibration offset value
  int shadowch;                         // what channel with ch3dewstrap shadow, 0=None, 1=channel1, 2=channel2, 3=manual, 4=tempprobe3
  int DisplayMode;                      // default display temp values in Celsius or Fahrenheight
  int displaytime;                      // interval in milliseconds (2000-5000 milliseconds) to wait between updating lcd and temps
  // Note, do not make this shorter else controller will become un-responsive as its always busy updating values
  // Do not make too large as the windows app has a minimum of 10s refresh, thus if you made this larger then
  // the values will be out of date. This time interval controls the refresh rate of updating the lcd displays
  // and calculating new values. Best for values between 2500 and 5000 (2.5s - 5s)
  int fantempoff;                       // the temperature at which the fan which be switched OFF
} dewconfig;

// ==============================================================================================
// CONDITIONAL DEFINES - DO NOT CHANGE ANYTHING IN THIS SECTION
#ifdef BLUETOOTH
SoftwareSerial btSerial( BTTX, BTRX);
char btline[MAXCOMMAND];
int bteoc;
int btidx;
#endif

#ifdef LCDDISPLAY
#ifdef LCD1602
int LCD1602Screen;                      // set the start page for LCD1602 display
#endif
#ifdef LCD1604
int LCD1604Screen;
#endif
#ifdef LCD2004
int LCD2004Screen;                      // set the start page for the LC2004 display, 4 pages, 2.5s delay (controlled by interval)
#endif
LiquidCrystal_I2C lcd(LCDADDRESS, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif

#ifdef OLEDDISPLAY
// Connect OLED VCC pin to 5V and OLED GND pin to GND
SSD1306AsciiWire myoled;
int DisplayPage;                        // set the start page for LCD1602 display
#endif

#ifdef DHTXX
dht mydht;                              // setup dhtxx sensor
int dhtchk;                             // variable to hold result of reading sensor
bool dhterrorflag;                      // state of DHTxx sensor read, true = Error, false = OK
#endif

#ifdef HTU21DXX                         // define HTU21D sensor and variables
HTU21D htu21d;                          // the HTU21D sensor, returns 998 is sensor not detected and 999 if CRC is bad
bool hval_error;                        // is relative humidity value valid?
bool tval_error;                        // is ambient temperature value valid?
bool dp_error;                          // is dew point calculation valid?
float hval_raw;                         // relative humidity from HTU21D sensor, raw - not compensated
float hval_comp;                        // relative humidity compensated for by temperature coefficient
// HTU21D Resolution settings - uses bit 7 and bit 0
const byte rh12t14 = B00000000;         // default on power on/reset, 50ms
const byte rh8t12  = B00000001;         // 8 bits for relative humidity and 12 bits for temperature, 3ms
const byte rh10t13 = B10000000;         // 25ms
const byte rh11t11 = B10000001;         // 8ms
byte myHTU21DResolution = rh12t14;      // resolution set to highest
const float TempCoefficient = -0.15;    // The temperature compensation coefficient value to ensure RH accuracy between 20-80%RH
#endif

#ifdef LCDDISPLAY
#ifdef LCD1602
char atstr[]    = "AT:";
char hustr[]    = "HU :";
char atbstr[]   = "ATB:";
char dpstr[]    = "DP:";
char c1str[]    = "C1:";
char c2str[]    = "C2:";
char c3str[]    = "C3:";
char p1str[]    = "P1:";
char p2str[]    = "P2:";
char p3str[]    = "P3:";
char tmstr[]    = "TM  :";
char tmostr[]   = "TMO :";
char fsstr[]    = "FS  :";
char pcbtstr[]  = "PCBT:";
char pcbsstr[]  = "PCBS:";
char pcbsstroff[]  = "PCBO:";
char ch3mstr[]  = "CH3M:";
char dashstr[]  = "--";
#endif
#ifdef LCD1604
char atstr[]    = "AT:";
char hustr[]    = "HU :";
char atbstr[]   = "ATB:";
char dpstr[]    = "DP:";
char c1str[]    = "C1:";
char c2str[]    = "C2:";
char c3str[]    = "C3:";
char p1str4[]   = "P1 :";
char p2str4[]   = "P2 :";
char p3str[]    = "P3 :";
char tmstr[]    = "TM  :";
char tmostr[]   = "TMO:";
char ch3mstr[]  = "CH3M:";
char fsstr[]    = "FS:";
char pcbtstr[]  = "PCBT:";
char pcbsstr[]  = "PCBS:";
char pcbsstroff[]  = "PCBO:";
char dashstr[]  = "--";
#endif
#ifdef LCD2004
char pcbtempstr[]         = "PCBTemp  = ";
char pcbtempsetpointstr[] = "PCBTarg  = ";
char fanspeedstr[]        = "Fanspeed = ";
char tmoffsetstr[]        = "TMOffset   = ";
char atstr1[]     = "AMBIENT TEMP= ";
char atbiasstr1[] = "AT Bias     = ";
char hustr1[]     = "REL HUMIDITY= ";
char dpstr1[]     = "DEW POINT   = ";
char tmstr1[]     = "TRACKING = ";        // tacking mode
char ch1str[]     = "CH1 ";
char ch2str[]     = "CH2 ";
char ch3str[]     = "CH3 ";
char tmpstr[]     = "TEMP = ";
char pwrstr[]     = "PWR  = ";
char modestr[]    = "MODE = ";
char fspeedstr1[] = "FAN SPEED= ";
char offstr[]     = "OFFSET = ";
char dashstr[]  = "--";
char pcbtstr[]  = "PCBT:";
char pcbsstr[]  = "PCBS:";
char pcbsstroff[]  = "PCBO:";
#endif
#endif

#ifdef OLEDDISPLAY
char programName1[]   = "myDCP3";      // Program title and version information
char programAuthor[]  = "(C) R BROWN 2017";
char dashstr[]  = "--";
#endif

// ==============================================================================================
// CODE START - CHANGE AT YOUR OWN RISK

void writeconfig()
{
  EEPROM_writeAnything(currentaddr, dewconfig);    // update values in EEPROM
}

void sendresponsestr(String str)
{
  if (Serial)
  {
    Serial.print(str);
  }
#ifdef BLUETOOTH
  {
    btSerial.print(str);
  }
#endif
}

void updatefanmotor()
{
  if ( dewconfig.fanspeed == POWER_100 )
  {
    analogWrite( FANMOTOR , 254 );      // set the PWM value to be 100%
    digitalWrite( RLED, HIGH );
    digitalWrite( GLED, LOW );
    digitalWrite( BLED, LOW );
  }
  else if ( dewconfig.fanspeed == POWER_0 )
  {
    analogWrite( FANMOTOR, 0 );         // turn fan off
    digitalWrite( RLED, LOW );
    digitalWrite( GLED, LOW );
    digitalWrite( BLED, LOW );
  }
  else if ( dewconfig.fanspeed == POWER_50 )
  {
    analogWrite( FANMOTOR, 220 );       // turn fan 50%
    digitalWrite( RLED, LOW );
    digitalWrite( GLED, LOW );
    digitalWrite( BLED, HIGH );
  }
  else if ( dewconfig.fanspeed == POWER_75 )
  {
    analogWrite( FANMOTOR, 240 );       // turn fan 75%
    digitalWrite( RLED, LOW );
    digitalWrite( GLED, HIGH );
    digitalWrite( BLED, LOW );
  }
}

int getpwr( float channeltemp, int trackmode )
{
  int pwrlevel = POWER_0;

  if ( trackmode == DEWPOINT )
  {
    if ( channeltemp >= (dew_point + 6.0 + dewconfig.offsetval) )
    {
      pwrlevel = POWER_0;
    }
    else if ( channeltemp >= (dew_point + 5.0 + dewconfig.offsetval) )
    {
      pwrlevel = POWER_10;
    }
    else if ( channeltemp >= (dew_point + 4.0 + dewconfig.offsetval) )
    {
      pwrlevel = POWER_20;
    }
    else if ( channeltemp >= (dew_point + 3.0 + dewconfig.offsetval) )
    {
      pwrlevel = POWER_50;
    }
    else if ( channeltemp >= (dew_point + 2.0 + dewconfig.offsetval) )
    {
      pwrlevel = POWER_75;
    }
    else pwrlevel = POWER_100;
  }
  else if ( trackmode == AMBIENT)             // assume trackmode is AMBIENT
  {
    if ( channeltemp <= (tval - 9.0 + dewconfig.offsetval) )
    {
      // is the OTA temperature way below ambient
      pwrlevel = POWER_100;                   // then set the pwr level to 100%
    }
    else if ( channeltemp <= (tval - 7.0 + dewconfig.offsetval) )
    {
      // is the OTA temperature 7 degrees or less below ambient
      pwrlevel = POWER_75;                    // then set the pwr level to 75%
    }
    else if ( channeltemp <= (tval - 5.0 + dewconfig.offsetval) )
    {
      // is the OTA temperature 5 degrees or less below ambient
      pwrlevel = POWER_50;                    // then set the pwr level to 50%
    }
    else if ( channeltemp <= (tval - 3.0 + dewconfig.offsetval) )
    {
      // is the OTA temperature 3 degrees or less below ambient
      pwrlevel = POWER_20;                    // then set the pwr level to 20%
    }
    else if ( channeltemp <= (tval - 1.0 + dewconfig.offsetval) )
    {
      // is the OTA temperature 1 degrees or less below ambient
      pwrlevel = POWER_10;                    // then set the pwr level to 10%
    }
    else pwrlevel = POWER_0;                  // if ota close to ambient then pwr level to 0%
  }                                           // end of if trackmode
  else if ( trackmode == HALFWAY)             // assume trackmode is MIDPOINT
  {
    if ( channeltemp >= (tval - (((tval - (int)dew_point) / 2) - 0) + dewconfig.offsetval))
    {
      pwrlevel = POWER_0;
    }
    else if ( channeltemp >= (tval - (((tval - (int)dew_point) / 2) - 2) + dewconfig.offsetval))
    {
      pwrlevel = POWER_20;
    }
    else if ( channeltemp >= (tval - (((tval - (int)dew_point) / 2) - 4) + dewconfig.offsetval))
    {
      pwrlevel = POWER_50;
    }
    else if ( channeltemp >= (tval - (((tval - (int)dew_point) / 2) - 6) + dewconfig.offsetval))
    {
      pwrlevel = POWER_100;
    }
    else pwrlevel = POWER_100;          // its lower than midpoint - 7
  }                                     // end of if trackmode
  else
  {
    pwrlevel = POWER_0;                 // set pwrlevel to 0 if there is an unknown tracking mode
  }
  return pwrlevel;
}

void seteepromdefaults()
{
  // set defaults because not found
  // data was erased so write some default values
  dewconfig.validdata = 99;
  dewconfig.TrackingState = DEWPOINT;
  dewconfig.offsetval = 0;
  dewconfig.fanspeed = 0;
  dewconfig.fantempon = 0;
  dewconfig.fantempoff = 0;
  dewconfig.ATBias = 0;
  dewconfig.ch1offset = 0.0;
  dewconfig.ch2offset = 0.0;
  dewconfig.ch3offset = 0.0;
  dewconfig.shadowch = 0;
  dewconfig.displaytime = 2500;         // allows user to set how long each page is displayed for
  dewconfig.DisplayMode = CELSIUS;
  updatefanmotor();
  writeconfig();                        // update values in EEPROM
}

// calculates dew point
// input:   humidity [%RH], temperature in C
// output:  dew point in C
float calc_dewpoint(float t, float h)
{
  float logEx, dew_point;
  logEx = 0.66077 + 7.5 * t / (237.3 + t) + (log10(h) - 2);
  dew_point = (logEx - 0.66077) * 237.3 / (0.66077 + 7.5 - logEx);
  return dew_point;
}

// process commands
void processcmd( )
{
  int len;
  String replystr = "";
  char mycmd;
  String param = "";

  replystr = queue.pop();
  len = replystr.length();
  if ( len == 2 )
  {
    mycmd = replystr.charAt(0);         // a valid command with no parameters, ie, 1#
  }
  else if ( len > 2 )
  {
    mycmd = replystr.charAt(0);         // this command has parameters
    param = replystr.substring(1, len - 1);
  }
  else return;

#ifdef DEBUG
  Serial.print("replystr = "); Serial.println(replystr);
  Serial.print("len = "); Serial.println(len);
  Serial.print("mycmd = "); Serial.println(mycmd);
  Serial.print("param = "); Serial.println(param);
#endif

  switch ( mycmd )
  {
    case 'v':       // v get version number
      replystr = "v" + String(ver) + endofstr;
      sendresponsestr(replystr);
      break;
    case '?':       // ? get the ch1offset and ch2offset and ch3offset values
      replystr = "?" + String(dewconfig.ch1offset) + hash + String(dewconfig.ch2offset) + hash + String(dewconfig.ch3offset) + endofstr;
      sendresponsestr(replystr);
      break;
    case 'E':      // E Returns which dewstrap channel the 3rd Dewstrap is shadowing (0-none, 1=channel1, 2=channel2, 3=manual, 4=tempprobe3)
      // then ch3 pwr and then ch3 temp
      replystr = "E" + String(dewconfig.shadowch) + endofstr;
      sendresponsestr(replystr);
      break;
    case 'g':      // g return the number of temperature probes
      {
        int nprobes = tprobe1 + tprobe2 + tprobe3;
        replystr = "g" + String(nprobes) + endofstr;
        sendresponsestr(replystr);
      }
      break;
    case 'T':      // T return tracking mode
      replystr = "T" + String(dewconfig.TrackingState) + endofstr;
      sendresponsestr(replystr);
      break;
    case 'F':      // F return fanspeed
      if ( dewconfig.fantempon > 0 )
      {
        if ( boardtemp >= dewconfig.fantempon )
        {
          replystr = "F100" + endofstr;
          sendresponsestr(replystr);
        }
        else
        {
          replystr = "F0" + endofstr;
          sendresponsestr(replystr);
        }
      }
      else
      {
        replystr = "F" + String(dewconfig.fanspeed) + endofstr;
        sendresponsestr(replystr);
      }
      break;
    case 'A':      // A return ambient temperature in Celsius
      replystr = "A";
#ifdef DHTXX
      if ( dhterrorflag == true )
#endif
#ifdef HTU21DXX
        if ( tval_error == true )
#endif
        {
          replystr = replystr + "0.0";
        }
        else
        {
          replystr = replystr + String(tval, 3);
        }
      replystr = replystr  + endofstr;
      sendresponsestr(replystr);
      break;
    case 'R':      // R return relative Humidity
      replystr = "R";
#ifdef DHTXX
      if ( dhterrorflag == true )
#endif
#ifdef HTU21DXX
        if ( hval_error == true )
#endif
        {
          replystr = replystr + "0";
        }
        else
        {
#ifdef DHTXX
          replystr = replystr + String((float)hval, 2);
#endif
#ifdef HTU21DXX
          replystr = replystr + String(hval_comp, 2);       // two decimal places
#endif
        }
      replystr = replystr + endofstr;
      sendresponsestr(replystr);
      break;
    case 'D':      // D return dewpoint temperature in Celsius
      replystr = "D";
#ifdef DHTXX
      if ( dhterrorflag == true )
#endif
#ifdef HTU21DXX
        if ( dp_error == true )
#endif
        {
          replystr = replystr + "0.0";
        }
        else
        {
          replystr = replystr + String(dew_point, 3);
        }
      replystr = replystr + endofstr;
      sendresponsestr(replystr);
      break;
    case 'C':      // C return ch1/ch2/ch3 temperature in Celsius
      replystr = "C" + String(ch1tempval, 3) + hash + String(ch2tempval, 3) + hash + String(ch3tempval, 3)  + endofstr;
      sendresponsestr(replystr);
      break;
    case 'W':      // W return ch1/ch2/ch3 power settings
      replystr = "W" + String(ch1pwrval) + hash + String(ch2pwrval) + hash + String(ch3pwrval) + endofstr;
      sendresponsestr(replystr);
      break;
    case 'B':      // B return AT Bias
      replystr = "B" + String(dewconfig.ATBias) + endofstr;
      sendresponsestr(replystr);
      break;
    case 'H':      // - Returns the lcddisplaytime
      replystr = "H" + String(dewconfig.displaytime) + endofstr;
      sendresponsestr(replystr);
      break;
    case  '1':
      if ( tprobe1 == 1 )               // only overrride if there is a probe
      {
        ch1override = 1;
        ch1pwrval = POWER_100;
        computeroverride = 1;
      }
      break;
    case '2':
      if ( tprobe2 == 1 )               // only overrride if there is a probe
      {
        ch2override = 1;
        ch2pwrval = POWER_100;
        computeroverride = 1;
      }
      break;
    case 'n':      // "n" switch to normal mode
      ch1override = 0;
      ch2override = 0;
      computeroverride = 0;
      break;
    case 'a':      // set tracking mode
      {
        // get tracking mode value as next parameter
        // extract the value from the command string
        dewconfig.TrackingState = param.toInt() & 0x03;
        writeconfig();
      }
      break;
    case 'h':      // get DisplayMode C or F
      replystr = "h" + String(dewconfig.DisplayMode) + endofstr;
      sendresponsestr(replystr);
      break;
    case 'c':      // "c" display in Celcius
      dewconfig.DisplayMode = CELSIUS;
      writeconfig();
      break;
    case 'f':      // "f" display in Fahrenheit
      dewconfig.DisplayMode = FAHRENHEIT;
      writeconfig();
      break;
    case '<':      // "<" decrease offset
      dewconfig.offsetval = dewconfig.offsetval - 1;
      if ( dewconfig.offsetval <= OFFSETNEGLIMIT )
      {
        dewconfig.offsetval = OFFSETNEGLIMIT;
      }
      writeconfig();
      break;
    case '>':      // " > " increase offset
      dewconfig.offsetval = dewconfig.offsetval + 1;
      if ( dewconfig.offsetval >= OFFSETPOSLIMIT )
      {
        dewconfig.offsetval = OFFSETPOSLIMIT;
      }
      writeconfig();
      break;
    case 'z':      // "z" zero offset
      dewconfig.offsetval = 0;
      writeconfig();
      break;
    case 'y':     // get tracking mode offset value
      replystr = "y" + String(dewconfig.offsetval) + endofstr;
      sendresponsestr(replystr);
      break;
    case 's':     // set fan speed
      {
        // get the next parameters
        // extract the value from the command string
        int fspeed = param.toInt();
        if ( fspeed < 0)
        {
          fspeed = 0;
        }
        if ( fspeed >= 100 )
        {
          fspeed = POWER_100;
        }
        dewconfig.fanspeed = fspeed;
        updatefanmotor();
        writeconfig();
      }
      break;
    case 'I':      // set fan temp control
      {
        // get the next parameters
        // extract the value from the command string
        int ftemp = param.toInt();
        if ( ftemp < 0)
        {
          ftemp = 0;
        }
        if ( ftemp >= 100 )
        {
          ftemp = POWER_100;
        }
        dewconfig.fantempon = ftemp;
        if ( ftemp == 0 )
        {
          dewconfig.fanspeed = POWER_0;
          updatefanmotor();
        }
        writeconfig();
      }
      break;
    case 'J':      // J return fantemp setting at which fan turns on
      replystr = "J" + String( dewconfig.fantempon ) + endofstr;
      sendresponsestr(replystr);
      break;
    case 'K':      // K return board temperature
      replystr = "K" + String( boardtemp ) + endofstr;
      sendresponsestr(replystr);
      break;
    case 'L':      // J return fantemp setting at which fan turns off
      replystr = "L" + String( dewconfig.fantempoff ) + endofstr;
      sendresponsestr(replystr);
      break;
    case 'M':      // set fan temp off
      {
        // get the next parameters
        // extract the value from the command string
        int ftemp = param.toInt();
        if ( ftemp < 0)
        {
          ftemp = 0;
        }
        if ( ftemp > 100 )
        {
          ftemp = 100;
        }
        if ( ftemp >= dewconfig.fantempon )
        {
          ftemp = dewconfig.fantempon - 2;
        }
        dewconfig.fantempoff = ftemp;
        writeconfig();
      }
      break;
    case 'e':      // set atbias
      {
        // get the next parameters
        // extract the value from the command string
        int biasnum = param.toInt();
        if ( biasnum < -4)
        {
          biasnum = -4;
        }
        if ( biasnum > 3)
        {
          biasnum = 3;
        }
        dewconfig.ATBias = biasnum;
        writeconfig();
      }
      break;
    case 'w':      // w write variables back to EEPROM
      writeconfig();
      break;
    case '[':      // [ set the ch1offset value
      // extract the value from the command string
      // convert to float;
      dewconfig.ch1offset = param.toFloat();
      writeconfig();
      break;
    case ']':      // ] set the ch2offset value
      // convert to float;
      dewconfig.ch2offset = param.toFloat();
      writeconfig();
      break;
    case '%':      // % set the ch3offset value
      // convert to float;
      dewconfig.ch3offset = param.toFloat();
      writeconfig();
      break;
    case '&':      // & clear ch1offset and ch2offset and ch3offset to 0.0
      dewconfig.ch1offset = 0.0;
      dewconfig.ch2offset = 0.0;
      dewconfig.ch3offset = 0.0;
      writeconfig();
      break;
    case '{':      // { turn off display
#ifdef LCDDISPLAY
      lcd.noDisplay();
      lcd.noBacklight();
#endif
#ifdef OLEDDISPLAY
      myoled.Display_Off();
#endif
      displayenabled = false;
      break;
    case '}':      // } turn on display
#ifdef LCDDISPLAY
      lcd.display();
      lcd.backlight();
#endif
#ifdef OLEDDISPLAY
      myoled.Display_On();
#endif
      displayenabled = true;
      break;
    case 'S':     // set how ch3 will behave
      {
        // set shadow dewstrap, 0 = off, 1=dewstrap1, 2=dewstrap3, 3=manual, 4=tempprobe3
        dewconfig.shadowch = param.toInt() & 0x07;
        switch ( dewconfig.shadowch)
        {
          case 0:                       // off
            ch3pwrval = 0;
            ch3tempval = 0.0;
            break;
          case 1:                       // ch1
            ch3pwrval = ch1pwrval;
            ch3tempval = ch1tempval;
            break;
          case 2:                       // ch2
            ch3pwrval = ch2pwrval;
            ch3tempval = ch2tempval;
            break;
          case 3:                       // manual
            // and save the pwrval for later recall
            ch3tempval = 0.0;           // temp not used for manual mode
            ch3pwrval = ch3manualpwrval;
            break;
          case 4:                       // use temp probe
            break;
        }
        writeconfig();
      }
      break;
    case 'G':     // 3rd dew channel set to manual mode
      {
        dewconfig.shadowch = 3;         // set to manual
        // set ch3pwrval
        int shadowval;
        shadowval = param.toInt();
        // shadowval is a percentage 0 to 100, when sent to dewstrap it is multiplied by 2.54
        if ( shadowval < 0 )
        {
          shadowval = 0;
        }
        if ( shadowval >= 100)
        {
          shadowval = 100;
        }
        ch3tempval = 0.0;               // temp not used in manual mode
        ch3pwrval = shadowval;
        ch3manualpwrval = shadowval;    // save for later recall if mode is switched
        writeconfig();
      }
      break;
    case 'b':     // set the displaytime bNum# where num is in milliseconds with range from 2500 t0 5000
      {
        int temptime;
        temptime = param.toInt();
        if ( temptime < MINPAGETIME )
        {
          temptime = MINPAGETIME;
        }
        if ( temptime > MAXPAGETIME)
        {
          temptime = MAXPAGETIME;
        }
        dewconfig.displaytime = temptime;
        writeconfig();
      }
      break;
    case 'r':
      seteepromdefaults();
      break;
      // any more commands place here
  }
  Serial.flush();                    // ensure serial buffer is empty
#ifdef BLUETOOTH
  btSerial.flush();
#endif
}

#ifdef BLUETOOTH
void clearbtPort()
{
  while (btSerial.available())
  {
    btSerial.read();
  }
}

void btSerialEvent()
{
  while (btSerial.available() && !bteoc )
  {
    char btinChar = btSerial.read();
    btline[btidx++] = btinChar;
    if (btidx >= MAXCOMMAND)
    {
      btidx = MAXCOMMAND - 1;
    }
    if (btinChar == '#')
    {
      bteoc = 1;
      btidx = 0;
      queue.push(String(btline));
      bteoc = 0;
      memset( btline, 0, MAXCOMMAND);
    }
  }
}
#endif

void clearSerialPort()
{
  while (Serial.available())
  {
    Serial.read();
  }
}

void serialEvent()
{
  // : starts the command, # ends the command, do not store these in the command buffer
  // read the command until the terminating # character
  while (Serial.available() && !eoc)
  {
    char inChar = (char) Serial.read();
    // add to string
    line[idx++] = inChar;
    if (idx >= MAXCOMMAND)
    {
      idx = MAXCOMMAND - 1;
    }
    if (inChar == '#')
    {
      eoc = 1;
      idx = 0;
      queue.push(String(line));
      eoc = 0;
      memset( line, 0, MAXCOMMAND);
    }
  }
}

// read the toggle switches and return which one is ON
int readtoggleswitches(int pinNum)
{
  //  sw1 650-720, sw2 310-380, sw1 and sw2 460-530
  int val = 0;                          // variable to store the read value
  val = analogRead(pinNum);             // read the input pin
  if ( val > 720 )
  {
    return 0;
  }
  if ( val >= 650 && val <= 720 )
  {
    return 1;                           // toggle sw1 ON and SW2 OFF
  }
  else if ( val >= 460 && val <= 530 )
  {
    return 3;                           // toggle sw1 and sw2 ON
  }
  else if ( val >= 310 && val <= 380 )
  {
    return 2;                           // toggle sw2 ON and SW1 OFF
  }
  else
  {
    return 0;                        // no button found to have been pushed
  }
}

#ifdef DHTXX
void updatedhtsensor()
{
  // Read the humidity and temperature from DHTxx sensor
  dhterrorflag = false;
  if ( MYSENSOR == DHT11 )
  {
    dhtchk = mydht.read11( DHTDATA );
  }
  else if ( MYSENSOR == DHT21 )
  {
    dhtchk = mydht.read21( DHTDATA );
  }
  else if ( MYSENSOR == DHT22 )
  {
    dhtchk = mydht.read22( DHTDATA );
  }
  else if ( MYSENSOR == DHT33 )
  {
    dhtchk = mydht.read33( DHTDATA );
  }
  else
  {
    dhtchk = -3;                     // error - undefined sensor type
  }

  if ( dhtchk == DHTLIB_OK )            // sensor is ok, so this is the main program now
  {
    hval = mydht.humidity;              // read the humidity
    tval = mydht.temperature + dewconfig.ATBias;  // Read the ambient temperature and add any calibration bias offset
    dew_point = calc_dewpoint(tval, hval);        //calculate dew point
  }
  else
  {
    dhterrorflag = true;                // do nothing as cannot read sensor
#ifdef LCDDISPLAY
    lcd.clear();
    lcd.print("DHTxx error");
#endif
#ifdef OLEDDISPLAY
    myoled.clear();
    myoled.InverseCharOn();
    myoled.println("DHTXX ERROR");
    myoled.InverseCharOff();
#endif
  }
}
#endif

#ifdef HTU21DXX
void read_htu21d_sensor()
{
  tval = htu21d.readTemperature();      // Read the ambient temperature
  if ( (tval == 998) || (tval == 999 ))
  {
    tval_error = true;
  }
  else
  {
    tval_error = false;                 // and tval = valid
    // add any calibration bias offset to ambient temperature
    // be careful - affects calculation below, offset for ATBIAS should not be necessary for HTU21D sensor
    tval = tval + dewconfig.ATBias;
  }

  hval_raw = htu21d.readHumidity();     // read the humidity, 998=timeout, 999=crc invalid
  if ( (hval_raw == 998) || (hval_raw == 999 ))
  {
    hval_error = true;
  }
  else
  {
    hval_error = false;                   // and hval = valid
    hval_comp = hval_raw + (25 - tval) * TempCoefficient;    // now perform temperature compensation using coefficient
  }

  if ( (hval_error == false) && (tval_error == false))
  {
    dew_point = calc_dewpoint(tval, hval_comp);    // calculate dew point only if both ambient temp and humidity are valid readings
    dp_error = false;
  }
  else
  {
    dp_error = true;                      // cannot calculate dewpoint if humidity or ambient temp is invalid
  }
}
#endif

// Request temp readings for ch1-ch3
void RequestTemperatures()
{
  if ( tprobe1 == 1 )
  {
    sensor1.requestTemperatures();
  }
  if ( tprobe2 == 1 )
  {
    sensor2.requestTemperatures();
  }
  if ( tprobe4 == 1 )                   // board temp - used to control fan
  {
    sensor4.requestTemperatures();
  }
}

// Read ch1-ch3 temperatures
void gettemps()
{
  if ( tprobe1 == 0 )
  {
    ch1tempval = 0.0;
    ch1oldtempval = ch1tempval;
  }
  else                                  // there is a ch1 probe
  {
    ch1tempval = sensor1.getTempCByIndex(0);        // get channel 1 temperature, always in celsius
    ch1tempval = ch1tempval + dewconfig.ch1offset;  // adjust temperature values by the offset
    if ( ch1override == 0 )             // override is off
    {
      ch1pwrval = getpwr( ch1tempval, dewconfig.TrackingState );
    }
    else
    {
      ch1pwrval = POWER_100;
    }
  }
  if ( tprobe2 == 0 )                   // do nothing but return
  {
    ch2tempval = 0.0;
    ch2oldtempval = ch2tempval;
  }
  else                                  // there is a ch2 probe
  {
    ch2tempval = sensor2.getTempCByIndex(0);        // get channel 2 temperature, always in celsius
    ch2tempval = ch2tempval + dewconfig.ch2offset;  // adjust temperature values by the offset
    if ( ch2override == 0 )             // override is off
    {
      ch2pwrval = getpwr( ch2tempval, dewconfig.TrackingState );
    }
    else
    {
      ch2pwrval = POWER_100;
    }
  }

  if ( tprobe4 == 0 )                               // do nothing but return
  {
    boardtemp = 0;
  }
  else                                              // there is a board probe
  {
    boardtemp = (int) sensor4.getTempCByIndex(0);   // get board temperature, always in celsius
  }

  if ( dewconfig.fantempon > 0 )                    // check if fan control is under fan temp sensor
  {
    if ( boardtemp >= dewconfig.fantempon )         // if board/case temp too high
    {
      pcbfanon = true;
      analogWrite( FANMOTOR , 254 );                // set the PWM value to be 100%
      digitalWrite( RLED, HIGH );
      digitalWrite( GLED, LOW );
      digitalWrite( BLED, LOW );
      dewconfig.fanspeed = POWER_100;
      updatefanmotor();
      writeconfig();
    }
    else
    {
      if ( pcbfanon == true )
      {
        if ( boardtemp < dewconfig.fantempoff )   // wait till pcb temp < dewconfig.fantempoff then turn fan off
        {
          analogWrite( FANMOTOR, 0 );             // turn fan off
          digitalWrite( RLED, LOW );
          digitalWrite( GLED, LOW );
          digitalWrite( BLED, LOW );
          dewconfig.fanspeed = POWER_0;
          updatefanmotor();
          writeconfig();
          pcbfanon = false;
        }
        else
        {
          // do nothing
        }
      }
    }
  }

  // do not check ch3 as it can be used to shadow ch1/ch2 and not use a temp probe!!!!

  // adjust ch1 and ch2 temperature values by the offset
  ch1tempval = ch1tempval + dewconfig.ch1offset;
  ch2tempval = ch2tempval + dewconfig.ch2offset;

  // determine the ch3 tempval
  switch ( dewconfig.shadowch )         // which mode is ch3?
  {
    case 0:                             // OFF
      ch3tempval = 0.0;
      break;
    case 1:                             // shadow ch1
      ch3tempval = ch1tempval;
      break;
    case 2:                             // shadow ch2
      ch3tempval = ch2tempval;
      break;
    case 3:                             // manual setting
      ch3tempval = 0.0;                 // ignore - use the setting of the slider from the main app
      break;
    case 4:                             // use temp probe3
      if ( tprobe3 == 1 )               // could be 0 if user switches to ch3=temp probe3
      {
        sensor3.requestTemperatures();
        delay(600 / (1 << (12 - TEMP_PRECISION)));      // should enough time to wait
        ch3tempval = sensor3.getTempCByIndex(0);        // get temp
        ch3tempval = ch3tempval + dewconfig.ch3offset;  // adjust by offset
      }
      else
      {
        ch3tempval = 0.0 + dewconfig.ch3offset;         // adjust by offset
      }
      break;
  }  // end of switch

  // now think about ch3, both ch1/ch2 have been updated so we can use the latest values
  // get channel 3 powerval
  switch ( dewconfig.shadowch )         // which mode is ch3?
  {
    case 0:                             // OFF
      ch3pwrval = POWER_0;
      break;
    case 1:                             // shadow ch1
      ch3pwrval = ch1pwrval;
      break;
    case 2:                             // shadow ch2
      ch3pwrval = ch2pwrval;
      break;
    case 3:                             // manual setting
      ch3pwrval = ch3manualpwrval;      // ignore - use the setting of the slider from the main app
      break;
    case 4:                             // use temp probe3
      if ( tprobe3 == 1 )
      {
        ch3pwrval = getpwr( ch3tempval, dewconfig.TrackingState );  // get pwr setting
      }
      else
      {
        ch3pwrval = POWER_0;
      }
      break;
  }  // end of switch

  analogWrite( CH1DEW, ch1pwrval * 2.54 );         // set the PWM value to be 0-254
  analogWrite( CH2DEW, ch2pwrval * 2.54 );
  analogWrite( CH3DEW, ch3pwrval * 2.54 );

  ch1oldtempval = ch1tempval;                       // remember last reading
  ch2oldtempval = ch2tempval;
  ch3oldtempval = ch3tempval;
}

#ifdef OLEDDISPLAY
void display1()
{
  /*
    SCREEN1
    Ambient
    Humidity
    DewPoint
    CH1 Temp
    CH1 Pwr
    CH1 Offset
    Tracking Mode
    AT Bias
  */

  // print Ambient Temperature, Humidity, Dew Point, Tracking Mode and Fan Speed to LCD
  String tempStr;
  float TempF;                          // used to hold conversion of temperatures to Fahrenheit

  myoled.print("AMBIENT : ");           // ambient air temperature
#ifdef DHTXX
  if ( dhterrorflag )
#endif
#ifdef HTU21DXX
    if ( tval_error == true )
#endif
      myoled.println(dashstr);
    else
    {
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        myoled.println(tval, 2);
      }
      else
      {
        TempF = (tval * 1.8) + 32;      // assume its Fahrenheit then
        myoled.println(TempF, 2);
      }
    }

  myoled.print("HUMIDITY: ");           // humdity
#ifdef DHTXX
  if ( dhterrorflag )
#endif
#ifdef HTU21DXX
    if ( hval_error == true )
#endif
    {
      myoled.println(dashstr);
    }
    else
    {
#ifdef DHTXX
      myoled.println(hval);
#endif
#ifdef HTU21DXX
      myoled.println(hval_comp, 2);
#endif
    }

  myoled.print("DEWPOINT: ");           // dewpoint
#ifdef DHTXX
  if ( dhterrorflag )
#endif
#ifdef HTU21DXX
    if ( dp_error == true )
#endif
    {
      myoled.println(dashstr);
    }
    else
    {
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        myoled.println(dew_point, 2);
      }
      else
      {

        TempF = (dew_point * 1.8) + 32; // assume its Fahrenheit then
        myoled.println(TempF, 2);
      }
    }

  myoled.print("CH1 TEMP: ");           // temperature probe 1 and power
  if ( tprobe1 != 1 )
  {
    myoled.println(dashstr);
  }
  else
  {
    if ( dewconfig.DisplayMode == CELSIUS )
    {
      myoled.println(ch1tempval, 2);
    }
    else
    {
      TempF = (ch1tempval * 1.8) + 32;  // assume its Fahrenheit then
      myoled.println(TempF, 2);
    }
  }
  myoled.print("CH1 PWR : ");
  myoled.println( ch1pwrval );

  myoled.print("CH1 OFF : ");           // ch1 offset
  myoled.println(dewconfig.ch1offset, 2);

  myoled.print("TRACKING: ");           // tracking mode, (A)mbient or (D)ew point or (M)id point
  if ( dewconfig.TrackingState == AMBIENT )
  {
    myoled.println(astr);
  }
  else if ( dewconfig.TrackingState == DEWPOINT )
  {
    myoled.println(dstr);
  }
  else if ( dewconfig.TrackingState == HALFWAY )
  {
    myoled.println(mstr);
  }

  myoled.print("AT BIAS : ");           // print ATBias
  myoled.println( dewconfig.ATBias );

  DisplayPage = 2;
}

void display2()
{
  /*
    SCREEN2
    CH2 Temp
    CH2 Pwr
    CH2 Offset
    CH3 Temp
    CH3 Pwr
    CH3 Offset
    CH3 Mode
  */

  String tempStr;
  float TempF;                          // used to hold conversion of temperatures to Fahrenheit

  myoled.print("CH2 TEMP: ");           // temperature probe 2 and power
  if ( tprobe2 != 1 )
  {
    myoled.println(dashstr);
  }
  else
  {
    if ( dewconfig.DisplayMode == CELSIUS )
    {
      myoled.println(ch2tempval, 2);
    }
    else
    {
      TempF = (ch2tempval * 1.8) + 32;  // assume its Fahrenheit then
      myoled.println(TempF, 2);
    }
  }

  myoled.print("CH2 PWR : ");
  myoled.println( ch2pwrval );

  myoled.print("CH2 OFF : ");           // ch2 offset
  myoled.println(dewconfig.ch2offset, 2);

  myoled.print("CH3 TEMP: ");           // probe3 temperature and power
  switch ( dewconfig.shadowch )
  {
    case 0:                             // none
      myoled.println(dashstr);
      break;
    case 1:                             // ch1
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        myoled.println(ch3tempval, 2);
      }
      else
      {
        TempF = (ch3tempval * 1.8) + 32;  // assume its Fahrenheit then
        myoled.println(TempF, 2);
      }
      break;
    case 2:                             // ch2
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        myoled.println(ch3tempval, 2);
      }
      else
      {
        TempF = (ch3tempval * 1.8) + 32;  // assume its Fahrenheit then
        myoled.println(TempF, 2);
      }
      break;
    case 3:                             // manual - ignore
      myoled.println(dashstr);
      break;
    case 4:                             // use probe3
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        if ( dewconfig.DisplayMode == CELSIUS )
        {
          myoled.println(ch3tempval, 2);
        }
        else
        {
          TempF = (ch3tempval * 1.8) + 32;  // assume its Fahrenheit then
          myoled.println(TempF, 2);
        }
      }
      break;
  }

  myoled.print("CH3 PWR : ");
  myoled.println( ch3pwrval );;

  myoled.print("CH3 OFF : ");           // ch3 offset
  myoled.println(dewconfig.ch3offset, 2);

  myoled.print("CH3 MODE: ");
  switch ( dewconfig.shadowch )
  {
    case 0: myoled.println(offmsgstr);
      break;
    case 1: myoled.println(ch1msgstr);
      break;
    case 2: myoled.println(ch2msgstr);
      break;
    case 3: myoled.println(manstr);
      break;
    case 4: myoled.println(probestr);
      break;
  }

  DisplayPage = 3;                      // set to display next page
}

void display3()
{
  /*
    SCREEN3
    PCBTemp
    PCBTarget
    FanSpeed
    Temp Mode
  */

  myoled.print("PCB TEMP : ");
  myoled.println(boardtemp);

  myoled.print("PCB ON  T: ");
  myoled.println(dewconfig.fantempon);

  myoled.print("PCB OFF T: ");
  myoled.println(dewconfig.fantempoff);

  myoled.print("FAN SPEED: ");
  switch ( dewconfig.fanspeed )
  {
    case 0: myoled.println(POWER_0);            // 0% = OFF
      break;
    case 50:
      myoled.println(POWER_50);                // 50%
      break;
    case 75:
      myoled.println(POWER_75);                // 75%
      break;
    case 100:
      myoled.println(POWER_100);                // 100%
      break;
  }

  myoled.print("TEMP MODE: ");
  if ( dewconfig.DisplayMode == CELSIUS )
  {
    myoled.println(celstr);
  }
  else
  {
    myoled.println(fstr);                   // assume its Fahrenheit then
  }

  DisplayPage = 1;                          // set to display next page
}
#endif

#ifdef LCDDISPLAY
#ifdef LCD1602
void updatelcd1602display1()
{
  /*
    SCREEN 1
    AT:       HU :
    DP:       ATB:
  */

  lcd.print(atstr);                     // AT: ambient air temperature
#ifdef DHTXX
  if ( dhterrorflag )
#endif
#ifdef HTU21DXX
    if ( tval_error == true )
#endif
    {
      lcd.print(dashstr);
    }
    else
    {
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(tval, 1);
      }
      else
      {
        TempF = (tval * 1.8) + 32;      // assume its Fahrenheit then
        lcd.print(TempF, 1);
      }
    }

  lcd.setCursor( 8, 0 );
  lcd.print(hustr);                     // HU: humdity
#ifdef DHTXX
  if ( dhterrorflag )
#endif
#ifdef HTU21DXX
    if ( hval_error == true )
#endif
    {
      lcd.print(dashstr);
    }
    else
    {
#ifdef DHTXX
      lcd.print( hval );
#endif
#ifdef HTU21DXX
      lcd.print(hval_comp, 0);
#endif
    }

  lcd.setCursor( 0, 1 );                // DP: dewpoint
  lcd.print(dpstr);
#ifdef DHTXX
  if ( dhterrorflag )
#endif
#ifdef HTU21DXX
    if ( dp_error == true )
#endif
    {
      lcd.print(dashstr);
    }
    else
    {
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(dew_point, 1);        // dew point C
      }
      else
      {
        TempF = (dew_point * 1.8) + 32; // assume its Fahrenheit then
        lcd.print(TempF, 1);
      }
    }

  // line 1 right
  lcd.setCursor( 8, 1 );
  lcd.print(atbstr);
  lcd.print(dewconfig.ATBias);

  LCD1602Screen = 2;                    // set to display next page
}

void updatelcd1602display2()
{
  /*
    SCREEN 2
    C1:     P1:
    C2:     P2:
  */

  // print ch1/ch2 temps, ch1/ch2 pwr to LCD
  // LCD1602 values are displayed as Integers
  lcd.print(c1str);                     // C1: temperature probe 1
  if ( tprobe1 == 1 )
  {
    if ( dewconfig.DisplayMode == CELSIUS )
    {
      lcd.print(ch1tempval, 1);
    }
    else
    {
      TempF = (ch1tempval * 1.8) + 32;  // assume its Fahrenheit then
      lcd.print(TempF, 1);
    }
  }
  else
  {
    lcd.print(dashstr);
  }

  lcd.setCursor(8, 0);                  // P1: probe1 power
  lcd.print(p1str);
  lcd.print(ch1pwrval);

  lcd.setCursor(0, 1);                  // C2: probe2 temperature
  lcd.print(c2str);
  if ( tprobe2 ==  1 )
  {
    if ( dewconfig.DisplayMode == CELSIUS )
    {
      lcd.print(ch2tempval, 1);
    }
    else
    {
      TempF = (ch2tempval * 1.8) + 32;  // assume its Fahrenheit then
      lcd.print(TempF, 1);
    }
  }
  else
  {
    lcd.print(dashstr);
  }
  lcd.setCursor(8, 1);                  // P2: probe2 power
  lcd.print(p2str);
  lcd.print(ch2pwrval);

  LCD1602Screen = 3;                    // set to display next page
}

void updatelcd1602display3()
{
  /*
    SCREEN 3
    C3:      P3:
    CH1O   CH2O    CH3O
  */

  // line0 Ch3 temp and Pwr  C3:xxx.x P3:xxx
  // line1 ch1offset, ch2offset, ch3offset
  lcd.print(c3str);                     // C3: probe3
  switch ( dewconfig.shadowch )
  {
    case 0:                             // none
      lcd.print(dashstr);
      break;
    case 1:                             // ch1
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(ch3tempval, 1 );
      }
      else
      {
        TempF = (ch3tempval * 1.8) + 32;
        lcd.print(TempF);
      }
      break;
    case 2:                             // ch2
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(ch3tempval, 1 );
      }
      else
      {
        TempF = (ch3tempval * 1.8) + 32;
        lcd.print(TempF, 1);
      }
      break;
    case 3:                             // manual - ignore
      lcd.print(dashstr);
      break;
    case 4:                             // use probe3
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(ch3tempval, 1);
      }
      else
      {
        TempF = (ch3tempval * 1.8) + 32;
        lcd.print(TempF, 1);
      }
      break;
  }

  lcd.setCursor(8, 0);                  // P3: probe3 power
  lcd.print(p3str);
  lcd.print(ch3pwrval);

  // print offsets for ch1-ch3
  // xx.xx xx.xx xx.xx
  lcd.setCursor(0, 1);
  if ( dewconfig.ch1offset > 0.0 )
  {
    lcd.print(plusstr);
  }
  lcd.print(dewconfig.ch1offset, 1);
  lcd.setCursor(5, 1);
  if ( dewconfig.ch2offset > 0.0 )
  {
    lcd.print(plusstr);
  }
  lcd.print(dewconfig.ch2offset, 1);
  lcd.setCursor(10, 1);
  if ( dewconfig.ch3offset > 0.0 )
  {
    lcd.print(plusstr);
  }
  lcd.print(dewconfig.ch3offset, 1);
  LCD1602Screen = 4;                    // set to display next page
}

void updatelcd1602display4()
{
  /*
    SCREEN 4
    TM  :A/M/D      TMO :
    PCBT:           PCBS:
  */

  // line 0
  lcd.print(tmstr);                     // TM: tracking mode, (A)mbient or (D)ew point or (M)id point
  if ( dewconfig.TrackingState == AMBIENT )
  {
    lcd.print(astr);
  }
  else if ( dewconfig.TrackingState == DEWPOINT )
  {
    lcd.print(dstr);
  }
  else if ( dewconfig.TrackingState == HALFWAY )
  {
    lcd.print(mstr);
  }

  lcd.setCursor(8, 0);                  // tracking mode offset
  lcd.print(tmostr);
  lcd.print(dewconfig.offsetval);

  // line 1
  // PCB Temp
  lcd.setCursor(0, 1);
  lcd.print(pcbtstr);
  lcd.print(boardtemp);
  lcd.setCursor(8, 1);
  lcd.print(pcbsstr);
  lcd.print(dewconfig.fantempon);

  LCD1602Screen = 5;
}

void updatelcd1602display5()
{
  /*
    SCREEN 5
    CH3M:
    FS:          Celsius/Fahren
  */

  // line0
  lcd.print(ch3mstr);                    // ch3 mode
  switch ( dewconfig.shadowch )
  {
    case 0: lcd.print(offmsgstr);
      break;
    case 1: lcd.print(c1str);
      break;
    case 2: lcd.print(c2str);
      break;
    case 3: lcd.print(manstr);
      break;
    case 4: lcd.print(probestr);
      break;
  }

  // line 1
  lcd.setCursor(0, 1);
  lcd.print(fsstr);                       // FanMotor speed
  switch ( dewconfig.fanspeed )
  {
    case 0: lcd.print(str0);           // 0% = OFF
      break;
    case 50:
      lcd.print(str50);                 // 50%
      break;
    case 75:
      lcd.print(str75);                 // 75%
      break;
    case 100:
      lcd.print(str100);                 // 100%
      break;
  }

  lcd.setCursor(8, 1);
  if ( dewconfig.DisplayMode == CELSIUS )
  {
    lcd.print(celstr);
  }
  else
  {
    lcd.print(fstr);                   // assume its Fahrenheit then
  }
  LCD1602Screen = 1;
}
#endif
#endif

#ifdef LCDDISPLAY
#ifdef LCD1604
void updatelcd1604display1()
{
  /*
    SCREEN 1
    AT: xx.xxc        Humidity
    DP: xx.xxc        ATBias
    CH1 = xx.xxc      P1:xxx%
    CH2 = xx.xxc      P2:xxx%
  */

  // line 0
  lcd.print(atstr);                     // AT: ambient air temperature
#ifdef DHTXX
  if ( dhterrorflag )
#endif
#ifdef HTU21DXX
    if ( tval_error == true )
#endif
    {
      lcd.print(dashstr);
    }
    else
    {
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(tval, 1);
      }
      else
      {
        TempF = (tval * 1.8) + 32;      // assume its Fahrenheit then
        lcd.print(TempF, 1);
      }
    }
  lcd.setCursor( 8, 0 );
  lcd.print(hustr);                     // HU: humdity
#ifdef DHTXX
  if ( dhterrorflag )
#endif
#ifdef HTU21DXX
    if ( hval_error == true )
#endif
    {
      lcd.print(dashstr);
    }
    else
    {
#ifdef DHTXX
      lcd.print( hval );
#endif
#ifdef HTU21DXX
      lcd.print(hval_comp, 0);
#endif
    }

  // line 1
  lcd.setCursor( 0, 1 );                // DP: dewpoint
  lcd.print(dpstr);
#ifdef DHTXX
  if ( dhterrorflag )
#endif
#ifdef HTU21DXX
    if ( dp_error == true )
#endif
    {
      lcd.print(dashstr);
    }
    else
    {
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(dew_point, 1);        // dew point C
      }
      else
      {
        TempF = (dew_point * 1.8) + 32; // assume its Fahrenheit then
        lcd.print(TempF, 1);
      }
    }

  lcd.setCursor( 8, 1 );
  lcd.print(atbstr);
  lcd.print(dewconfig.ATBias);

  // line 2
  lcd.setCursor(0, 2);
  lcd.print(c1str);                     // C1: temperature probe 1
  if ( tprobe1 == 1 )
  {
    if ( dewconfig.DisplayMode == CELSIUS )
    {
      lcd.print(ch1tempval, 1);
    }
    else
    {
      TempF = (ch1tempval * 1.8) + 32;  // assume its Fahrenheit then
      lcd.print(TempF, 1);
    }
  }
  else
  {
    lcd.print(dashstr);
  }

  lcd.setCursor(8, 2);                  // P1: probe1 power
  lcd.print(p1str4);
  lcd.print(ch1pwrval);

  // line 3
  lcd.setCursor(0, 3);                  // C2: probe2 temperature
  lcd.print(c2str);
  if ( tprobe2 ==  1 )
  {
    if ( dewconfig.DisplayMode == CELSIUS )
    {
      lcd.print(ch2tempval, 1);
    }
    else
    {
      TempF = (ch2tempval * 1.8) + 32;  // assume its Fahrenheit then
      lcd.print(TempF, 1);
    }
  }
  else
  {
    lcd.print(dashstr);
  }
  lcd.setCursor(8, 3);                  // P2: probe2 power
  lcd.print(p2str4);
  lcd.print(ch2pwrval);


  LCD1604Screen = 2;                    // set to display next page
}

void updatelcd1604display2()
{
  /*
    SCREEN 2
    CH1O   CH2O     CH3O
    CH3 = xx.xxc    P3:xxx%
    TM A/M/D        TMO:xx
    CH3M: xxxxx     FS: xxx
  */

  // line0
  // print offsets for ch1-ch3
  // xx.xx xx.xx xx.xx
  if ( dewconfig.ch1offset > 0.0 )
  {
    lcd.print(plusstr);
  }
  lcd.print(dewconfig.ch1offset, 1);
  lcd.setCursor(5, 0);
  if ( dewconfig.ch2offset > 0.0 )
  {
    lcd.print(plusstr);
  }
  lcd.print(dewconfig.ch2offset, 1);
  lcd.setCursor(10, 0);
  if ( dewconfig.ch3offset > 0.0 )
  {
    lcd.print(plusstr);
  }
  lcd.print(dewconfig.ch3offset, 1);

  lcd.setCursor(0, 1);
  lcd.print(c3str);                     // C3: probe3
  switch ( dewconfig.shadowch )
  {
    case 0:                             // none
      lcd.print(dashstr);
      break;
    case 1:                             // ch1
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(ch3tempval, 1 );
      }
      else
      {
        TempF = (ch3tempval * 1.8) + 32;
        lcd.print(TempF);
      }
      break;
    case 2:                             // ch2
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(ch3tempval, 1 );
      }
      else
      {
        TempF = (ch3tempval * 1.8) + 32;
        lcd.print(TempF, 1);
      }
      break;
    case 3:                             // manual - ignore
      lcd.print(dashstr);
      break;
    case 4:                             // use probe3
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(ch3tempval, 1);
      }
      else
      {
        TempF = (ch3tempval * 1.8) + 32;
        lcd.print(TempF, 1);
      }
      break;
  }

  lcd.setCursor(9, 1);                  // P3: probe3 power
  lcd.print(p3str);
  lcd.print(ch3pwrval);


  // line2
  lcd.setCursor(0, 2);
  lcd.print(tmstr);                     // TM: tracking mode, (A)mbient or (D)ew point or (M)id point
  if ( dewconfig.TrackingState == AMBIENT )
  {
    lcd.print(astr);
  }
  else if ( dewconfig.TrackingState == DEWPOINT )
  {
    lcd.print(dstr);
  }
  else if ( dewconfig.TrackingState == HALFWAY )
  {
    lcd.print(mstr);
  }

  lcd.setCursor(9, 2);                  // tracking mode offset
  lcd.print(tmostr);
  lcd.print(dewconfig.offsetval);

  // line3
  lcd.setCursor(0, 3);
  lcd.print(ch3mstr);                    // ch3 mode
  switch ( dewconfig.shadowch )
  {
    case 0: lcd.print(offmsgstr);
      break;
    case 1: lcd.print(c1str);
      break;
    case 2: lcd.print(c2str);
      break;
    case 3: lcd.print(manstr);
      break;
    case 4: lcd.print(probestr);
      break;
  }
  lcd.setCursor(9, 3);
  lcd.print(fsstr);                       // FanMotor speed
  switch ( dewconfig.fanspeed )
  {
    case 0: lcd.print(str0);           // 0% = OFF
      break;
    case 50:
      lcd.print(str50);                 // 50%
      break;
    case 75:
      lcd.print(str75);                 // 75%
      break;
    case 100:
      lcd.print(str100);                 // 100%
      break;
  }

  LCD1604Screen = 3;                    // set to display next page
}

void updatelcd1604display3()
{
  /*
    SCREEN 3
    PCBT: xxc       PCBST: xxc
  */

  // line 0
  lcd.print(pcbtstr);                   // pcb temp
  lcd.print(boardtemp);
  lcd.setCursor(8, 0);
  lcd.print(pcbsstr);                   // pcb fan temp on
  lcd.print(dewconfig.fantempon);
  lcd.setCursor(0, 1);
  lcd.print(pcbsstroff);                // pcb fan temp off
  lcd.print(dewconfig.fantempoff);

  // line1

  LCD1604Screen = 1;
}
#endif
#endif

#ifdef LCDDISPLAY
#ifdef LCD2004
void updatelcd2004display1()
{
  /*
    SCREEN 1
    AMBIENT TEMP = xx.xx
    DEW POINT    = xx.xx
    REL HUMIDITY = xx
    AT BIAS      = xx
  */

  // LCD2004 values are displayed to 2 decimal places

  lcd.print(atstr1);                    // ambient temperature
#ifdef DHTXX
  if ( dhterrorflag )
#endif
#ifdef HTU21DXX
    if ( tval_error == true )
#endif
    {
      lcd.print(dashstr);
    }
    else
    {
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(tval, 2);
      }
      else
      {
        TempF = (tval * 1.8) + 32;
        lcd.print(TempF, 2);
      }
    }
  lcd.setCursor( 0, 1);                 // row 1
  lcd.print(dpstr1);                    // dewpoint
#ifdef DHTXX
  if ( dhterrorflag )
#endif
#ifdef HTU21DXX
    if ( dp_error == true )
#endif
    {
      lcd.print(dashstr);
    }
    else
    {
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(dew_point, 2);        // dew point C
      }
      else
      {
        TempF = (dew_point * 1.8) + 32;
        lcd.print(TempF, 2);
      }
    }
  lcd.setCursor( 0, 2 );
  lcd.print(hustr1);                    // humdity
#ifdef DHTXX
  if ( dhterrorflag )
#endif
#ifdef HTU21DXX
    if ( hval_error == true )
#endif
    {
      lcd.print(dashstr);
    }
    else
    {
#ifdef DHTXX
      lcd.print( hval );
#endif
#ifdef HTU21DXX
      lcd.print(hval_comp, 2);
#endif
    }

  // line 3
  lcd.setCursor(0, 3);
  lcd.print(atbiasstr1);                // ATBias
  lcd.print(dewconfig.ATBias);

  LCD2004Screen = 2;
}

void updatelcd2004display2()
{
  /* SCREEN 2
    CH1 TEMP = xx.xx
    CH2 TEMP = xx.xx
    CH1 PWR = xxx
    CH2 PWR = xxx
  */

  lcd.print(ch1str);                    // ch1 temperature
  lcd.print(tmpstr);
  if ( tprobe1 == 0 )
  {
    lcd.print(dashstr);
  }
  else
  {
    if ( dewconfig.DisplayMode == CELSIUS )
    {
      lcd.print(ch1tempval, 2 );
    }
    else
    {
      TempF = (ch1tempval * 1.8) + 32;
      lcd.print(TempF, 2);
    }
  }

  // line 1
  lcd.setCursor( 0 , 1 );               // next line
  lcd.print(ch2str);                    // ch2 temperature
  lcd.print(tmpstr);
  if ( tprobe2 == 0 )
  {
    lcd.print(dashstr);
  }
  else
  {
    if ( dewconfig.DisplayMode == CELSIUS )
    {
      lcd.print(ch2tempval, 2 );
    }
    else
    {
      TempF = (ch2tempval * 1.8) + 32;
      lcd.print(TempF, 2);
    }
  }

  // line 2
  lcd.setCursor( 0 , 2 );               // next line
  lcd.print(ch1str);                    // ch1 power
  lcd.print(pwrstr);
  lcd.print(ch1pwrval);

  // line 3
  lcd.setCursor( 0 , 3 );               // ch2 power
  lcd.print(ch2str);
  lcd.print(pwrstr);
  lcd.print(ch2pwrval);

  LCD2004Screen = 3;
}

void updatelcd2004display3()
{
  /* SCREEN 3
    CH3 TEMP = xx.xx
    CH3 PWR  = xxx
    CH3 MODE = OFF/1/2/3/MAN/PROBE
    TRACKING = D/M/A
  */

  // line 0
  lcd.print(ch3str);                    // ch3 temperature
  lcd.print(tmpstr);
  switch ( dewconfig.shadowch )
  {
    case 0:                             // none
      lcd.print(dashstr);
      break;
    case 1:                             // ch1
    case 2:                             // ch2
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(ch3tempval, 2 );
        lcd.print(celstr);
      }
      else
      {
        TempF = (ch3tempval * 1.8) + 32;
        lcd.print(TempF, 2);
        lcd.print(fstr);
      }
      break;
    case 3:                             // manual - ignore
      lcd.print(dashstr);
      break;
    case 4:                             // use probe3
      if ( dewconfig.DisplayMode == CELSIUS )
      {
        lcd.print(ch3tempval, 2 );
        lcd.print(celstr);
      }
      else
      {
        TempF = (ch3tempval * 1.8) + 32;
        lcd.print(TempF, 2);
        lcd.print(fstr);
      }
      break;
  }
  // line 1
  lcd.setCursor( 0 , 1 );
  lcd.print(ch3str);                    // ch3 power
  lcd.print(pwrstr);
  lcd.print(ch3pwrval);

  // line 2
  lcd.setCursor( 0 , 2 );
  lcd.print(ch3str);                    // ch3 mode
  lcd.print(modestr);
  switch ( dewconfig.shadowch )
  {
    case 0: lcd.print(offstr);
      break;
    case 1: lcd.print(ch1str);
      break;
    case 2: lcd.print(ch2str);
      break;
    case 3: lcd.print(manstr);
      break;
    case 4: lcd.print(probestr);
      break;
  }

  // line 3
  lcd.setCursor( 0 , 3 );
  lcd.print(tmstr1);                    // tracking mode, (A)mbient or (D)ew point
  if ( dewconfig.TrackingState == AMBIENT )
    lcd.print("AMBIENT");
  else if ( dewconfig.TrackingState == DEWPOINT )
    lcd.print("DEWPOINT");
  else if ( dewconfig.TrackingState == HALFWAY )
    lcd.print("MIDPOINT");

  LCD2004Screen = 4;
}

void updatelcd2004display4()
{
  /* SCREEN 4
    CH1 OFFSET = +-x.x
    CH2 OFFSET = +-x.x
    CH3 OFFSET = +-x.x
    TMOffset   =
  */

  // line 0
  lcd.setCursor(0, 0);
  lcd.print(ch1str);                    // ch1 offset
  lcd.print(offstr);
  if (dewconfig.ch1offset > 0 )
    lcd.print("+");
  lcd.print(dewconfig.ch1offset);

  // line 1
  lcd.setCursor(0, 1);
  lcd.print(ch2str);                    // ch2 offset
  lcd.print(offstr);
  if (dewconfig.ch2offset > 0 )
  {
    lcd.print(plusstr);
  }
  lcd.print(dewconfig.ch2offset);

  // line 2
  lcd.setCursor(0, 2);
  lcd.print(ch3str);                    // ch3 offset
  lcd.print(offstr);
  if (dewconfig.ch3offset > 0 )
  {
    lcd.print(plusstr);
  }
  lcd.print(dewconfig.ch3offset);

  // line 3
  lcd.setCursor(0, 3);
  lcd.print(tmoffsetstr);
  lcd.print(dewconfig.offsetval);

  LCD2004Screen = 5;
}

void updatelcd2004display5()
{
  /* SCREEN 5
    PCBTemp  = x.xx
    PCBTarg  = xx.x
    Fanspeed = xxx
  */

  // line 0
  lcd.print(pcbtempstr);
  lcd.print(boardtemp);

  // line 1
  lcd.setCursor(0 , 1);
  lcd.print(pcbtempsetpointstr);
  lcd.print(dewconfig.fantempon);

  // line 2
  lcd.setCursor( 0, 2 );
  lcd.print(fanspeedstr);                 // FanMotor speed
  switch ( dewconfig.fanspeed )
  {
    case 0: lcd.print(str0);              // 0% = OFF
      break;
    case 50:
      lcd.print(str50);                   // 50%
      break;
    case 75:
      lcd.print(str75);                   // 75%
      break;
    case 100:
      lcd.print(str100);                  // 100%
      break;
  }

  LCD2004Screen = 1;
}
#endif
#endif

void setup()
{
  int datasize;                         // will hold size of the struct dewconfig in bytes, 18 bytes
  int nlocations;                       // number of storage locations available in EEPROM
  bool found;                           // did we find any stored values?

  Serial.begin(SERIALPORTSPEED);        // start serial port now
  clearSerialPort();                    // clear any garbage from serial buffer
#ifdef BLUETOOTH
  btSerial.begin(BTPORTSPEED);          // do not change!!!!!!!!!!!!
  clearbtPort();
#endif

  displayenabled = true;
  temprefresh = false;

#ifdef LCDDISPLAY
#ifdef LCD1602
  lcd.begin(16, 2);
#endif
#ifdef LCD1604
  lcd.begin(16, 4);
#endif
#ifdef LCD2004
  lcd.begin(20, 4);
#endif
  lcd.setBacklight(HIGH);
  lcd.print("myDewCtrlrPro3");
  lcd.setCursor( 0, 1 );                // col, row
  lcd.print("(c)RBB ");
  lcd.print(ver);
#ifdef LCD1602
  LCD1602Screen = 1;                    // always start at page 1 for any display
#endif
#ifdef LCD1604
  LCD1604Screen = 1;
#endif
#ifdef LCD2004
  LCD2004Screen = 1;
#endif
#endif

#ifdef OLEDDISPLAY
  Wire.begin();
  myoled.begin(&Adafruit128x64, I2C_ADDRESS);
  myoled.set400kHz();
  myoled.setFont(Adafruit5x7);

  myoled.clear();
  myoled.Display_Normal();              // black on white
  myoled.Display_On();
  myoled.Display_Rotate(0);             // portrait, not rotated
  myoled.Display_Bright();
  DisplayPage = 1;                      // start at page1
  // print startup screen
  // The screen size is 128 x 64, so using characters at 6x8 this gives 21chars across and 8 lines down
  myoled.println(programName1);
  myoled.println(ver);
  myoled.InverseCharOn();
  myoled.println(programAuthor);
  myoled.InverseCharOff();
#endif

  pinMode( CH1DEW, OUTPUT );            // pwm outputs for dew straps
  pinMode( CH2DEW, OUTPUT );
  pinMode( CH3DEW, OUTPUT );
  pinMode( FANMOTOR, OUTPUT );          // pwm output for 12vdc fan cia TIP120
  pinMode( RLED, OUTPUT );              // setup the RGB LEDS - these are analog pins in V3, treated like DigitalOut pins
  pinMode( GLED, OUTPUT );
  pinMode( BLED, OUTPUT );

  eoc = 0;
  idx = 0;
  memset(line, 0, MAXCOMMAND);
#ifdef BLUETOOTH
  bteoc = 0;
  btidx = 0;
  memset(btline, 0, MAXCOMMAND);
#endif

  currentaddr = 0;                      // start at 0 if not found later
  found = false;
  writenow = false;
  datasize = sizeof( dewconfig );      // 26 bytes
  nlocations = EEPROMSIZE / datasize;  // for AT328P = 1024 / datasize = 39 locations

  for (int lp1 = 0; lp1 < nlocations; lp1++ )
  {
    int addr = lp1 * datasize;
    EEPROM_readAnything( addr, dewconfig );
    if ( dewconfig.validdata == 99 )    // check to see if the data is valid
    {
      currentaddr = addr;               // data was erased so write some default values
      found = true;
      break;
    }
  }
  if ( found == true )
  {
    // restore the dew controller settings
    // done after this in one hit
    // mark current eeprom address as invalid and use next one
    // each time focuser starts it will read current storage, set it to invalid, goto next location and
    // write values to there and set it to valid - so it doesnt always try to use same locations over and
    // over and destroy the eeprom
    // using it like an array of [0-nlocations], ie 100 storage locations for 1k EEPROM
    EEPROM_readAnything( currentaddr, dewconfig );
    dewconfig.validdata = 0;
    writeconfig();                      // update values in EEPROM
    currentaddr += datasize;            // goto next free address and write data
    // bound check the eeprom storage and if greater than last index [0-EEPROMSIZE-1] then set to 0
    if ( currentaddr >= (nlocations * datasize) )
    {
      currentaddr = 0;
    }
    dewconfig.validdata = 99;
    writeconfig();                      // update values in EEPROM
  }
  else
  {
    seteepromdefaults();                // set defaults because not found
  }

  temptimer = displaytimer = millis();  // start time interval for display and temperature updates

#ifdef DHTXX
  dhterrorflag = false;
  updatedhtsensor();                    // trigger humidity and ambient and calc dew_point
#endif

#ifdef HTU21DXX
  // check for HTU21D sensor
  Wire.begin();                         // prime the single wire interface
  htu21d.softReset();                   // this should reset the HT sensor
  delay(20);                            // wait enough time for reset to end, should take 15ms
  htu21d.begin();                       // the default start resolution is meant to be 12bit
  htu21d.setResolution(myHTU21DResolution);    // specifically set resolution to 12bit just to be sure
  hval_error = false;                   // if there was an error reading humidity value then set to true
  tval_error = false;                   // if there was an error reading temperature value then set to true
  dp_error = false;                     // if humidity or ambient temp is invalid then cannot calculate dew point
  read_htu21d_sensor();                 // trigger humidity and ambient and calc dew_point
#endif

  // start temperature sensors DS18B20
  tprobe1 = 0;                                // set probe indicators OFF
  tprobe2 = 0;
  tprobe3 = 0;
  tprobe4 = 0;
  sensor1.begin();                            // start the temperature sensor1
  sensor1.getDeviceCount();                   // should return 1 if probe connected
  for (int i = 0; i < MAXPROBES; i++)         // look for probes
  {
    if (sensor1.getAddress(tpAddress, i))     // Search the wire for address
    {
      // found?
      tprobe1 = 1;                            // there is a probe1
      sensor1.setResolution(tpAddress, TEMP_PRECISION);
      // set the resolution for that probe to 9bit 0.5degC
      // as the accuracy is only +-0.5degC anyway
    }
  }

  // repeat for channel 2 probe
  sensor2.begin();
  sensor2.getDeviceCount();                   // should return 1 if probe connected
  for (int i = 0; i < MAXPROBES; i++)
  {
    if (sensor2.getAddress(tpAddress, i))     // Search the wire for address
    {
      tprobe2 = 1;                            // there is a probe2
      sensor2.setResolution(tpAddress, TEMP_PRECISION);
    }
  }

  // repeat for channel 3 probe
  sensor3.begin();
  sensor3.getDeviceCount();                   // should return 1 if probe connected
  for (int i = 0; i < MAXPROBES; i++)
  {
    if (sensor3.getAddress(tpAddress, i))     // Search the wire for address
    {
      tprobe3 = 1;                            // there is a probe3
      sensor3.setResolution(tpAddress, TEMP_PRECISION);
    }
  }

  // repeat for channel 4 board temp sensor
  pcbfanon = false;
  sensor4.begin();
  sensor4.getDeviceCount();                   // should return 1 if probe connected
  for (int i = 0; i < MAXPROBES; i++)
  {
    if (sensor4.getAddress(tpAddress, i))     // Search the wire for address
    {
      tprobe4 = 1;                            // there is a probe4
      sensor4.setResolution(tpAddress, TEMP_PRECISION);
    }
  }

  int nprobes = tprobe1 + tprobe2 + tprobe3;
#ifdef LCDDISPLAY
  lcd.clear();
  lcd.setCursor( 0, 0 );
  lcd.print(probesequals);                    // print the number of temperature probes
  lcd.print(nprobes);
  lcd.setCursor( 0, 1 );
  lcd.print(PCBprobeequalsstr);
  if ( tprobe4 == 1 )
  {
    lcd.print(yesstr);
  }
  else
  {
    lcd.print(nostr);
  }
#endif
#ifdef OLEDDISPLAY
  myoled.print(probesequals);
  myoled.println(nprobes);
  myoled.print(PCBprobeequalsstr);
  if ( tprobe4 == 1 )
  {
    myoled.print(yesstr);
  }
  else
  {
    myoled.print(nostr);
  }
#endif

  ch1pwrval = 0;                        // set power to dew straps to 0 and write to dew straps
  ch2pwrval = 0;
  ch3pwrval = 0;
  computeroverride = 0;
  ch1override = 0;
  ch2override = 0;
  ch1tempval = 0.0;
  ch2tempval = 0.0;
  ch3tempval = 0.0;
  boardtemp = 0;
  ch1oldtempval = ch1tempval;
  ch2oldtempval = ch2tempval;
  ch3oldtempval = ch3tempval;
  analogWrite( CH1DEW, ch1pwrval );     // set dewchannel1, 2, 3 off
  analogWrite( CH2DEW, ch2pwrval );
  analogWrite( CH3DEW, ch3pwrval );

  buttonLastChecked = millis() + BUTTONDELAY; // force a check this cycle

  if ( (dewconfig.fanspeed > 100) || (dewconfig.fanspeed < 0) )
  {
    dewconfig.fanspeed = POWER_0;
  }

  if ( dewconfig.fantempon == 0 )
  {
    dewconfig.fanspeed = POWER_0;
    updatefanmotor();
  }

  RequestTemperatures();
  delay(1000);
  gettemps();                        // read ch1/ch2/ch3 temperatures

  // this bit of code takes care of the fact that first run this variable has not been set yet
  dewconfig.displaytime = (dewconfig.displaytime < MINPAGETIME) ? MINPAGETIME : dewconfig.displaytime;
  dewconfig.displaytime = (dewconfig.displaytime > MAXPAGETIME) ? MAXPAGETIME : dewconfig.displaytime;
  writeconfig();
}

// --------------------------------------------------------
void loop()
{
#ifdef BLUETOOTH
  btSerialEvent();                      // check for command from bt adapter
#endif

  if ( queue.count() >= 1 )             // check for serial command
  {
    processcmd();
  }

  // check toggle switch for override
  buttonchecktime = millis();
  // make sure 1s has elapsed since last read
  if ( ((buttonchecktime - buttonLastChecked) > BUTTONDELAY) || (buttonchecktime < buttonLastChecked) )
  {
    if ( computeroverride == 0 )                        // if the windows app does not have control
    {
      PBVal = readtoggleswitches(TOGGLESWPIN);          // read the toggle switches
      buttonLastChecked = buttonchecktime;              // reset the lastChecked value
      switch ( PBVal )                                  // then use the state of the switches
      {
        case 0:                         // toggle sw1 and sw2 are OFF
          if ( ch1override == 1)
          {
            ch1override = 0;
          }
          if ( ch2override == 1)
          {
            ch2override = 0;
          }
          break;
        case 1:                         // toggle sw1 is ON and 2 is off
          if ( tprobe1 == 1 )           // if there is a probe1
          {
            ch1override = 1;            // set ch1 to override
            ch1pwrval = POWER_100;            // set pwm pwr to 100%
          }
          if ( ch2override == 1)
          {
            ch2override = 0;
          }        break;
        case 2:                         // toggle sw2 is ON and SW1 is OFF
          if ( tprobe2 == 1 )           // if there is a probe1
          {
            ch2override = 1;            // set ch2 to override
            ch2pwrval = POWER_100;            // set pwm pwr to 100%
          }
          if ( ch1override == 1)
          {
            ch1override = 0;
          }
          break;
        case 3:                         // toggle sw1 and sw2 are ON
          if ( tprobe1 == 1 )           // if there is a probe1
          {
            ch1override = 1;            // set ch1 to override
            ch1pwrval = POWER_100;            // set pwm pwr to 100%
          }
          if ( tprobe2 == 1 )           // if there is a probe2
          {
            ch2override = 1;            // set ch2 to override
            ch2pwrval = POWER_100;            // set pwm pwr to 100%
          }
          break;
      }
    }
    else                                // computeroverride is true so ignore reading switches as its under control of windows app
    {
      // do nothing
    }
  }
  // end of toggle-switch code

  // handle the temperatures
  currenttime = millis();
  if ( ((currenttime - temptimer) > TEMPUPDATES) || (currenttime < temptimer) )
  {
    temptimer = currenttime;     // update the timestamp
    if ( temprefresh == false )
    {
      // trigger reads of temperature probes
      RequestTemperatures();
      temprefresh = true;
    }
    else
    {
      // read the temperature probes
      gettemps();
#ifdef DHTXX
      updatedhtsensor();                  // trigger humidity and ambient and calc dew_point
#endif
#ifdef HTU21DXX
      read_htu21d_sensor();               // read humidity and ambient and calc dew_point
#endif
      temprefresh = false;
    }
  }

  // Handle the lcd display
  currenttime = millis();
  if ( displayenabled == true )        // if display is off then do not print values
  {
    if ( ((currenttime - displaytimer) > dewconfig.displaytime) || (currenttime < displaytimer) )
    {
      displaytimer = currenttime;     // update the timestamp
#ifdef LCDDISPLAY
      lcd.clear();
#ifdef LCD1602
      switch ( LCD1602Screen )
      {
        case 1: updatelcd1602display1();
          break;
        case 2: updatelcd1602display2();
          break;
        case 3: updatelcd1602display3();
          break;
        case 4: updatelcd1602display4();
          break;
        case 5: updatelcd1602display5();
          break;
      }
#endif
#ifdef LCD1604
      switch ( LCD1604Screen )
      {
        case 1: updatelcd1604display1();
          break;
        case 2: updatelcd1604display2();
          break;
        case 3: updatelcd1604display3();
          break;
      }
#endif
#ifdef LCD2004
      switch ( LCD2004Screen )
      {
        case 1: updatelcd2004display1();
          break;
        case 2: updatelcd2004display2();
          break;
        case 3: updatelcd2004display3();
          break;
        case 4: updatelcd2004display4();
          break;
        case 5: updatelcd2004display5();
          break;
      }
#endif
#endif
#ifdef OLEDDISPLAY
      myoled.clear();
      switch ( DisplayPage )
      {
        case 1: display1();
          break;
        case 2: display2();
          break;
        case 3: display3();
          break;
      }
#endif
    }
  }
}

// FIRMWARE CODE END
// ==============================================================================================
