#ifndef Settings_h
#define Settings_h

// GLOBAL DEFINITIONS START - DO NOT CHANGE
#define DHT11  1
#define DHT21  2
#define DHT22  3
#define DHT33  4

#define SERIALPORTSPEED   57600
#define BTPORTSPEED       9600                    // define Bluetooth Speed
#define I2C_ADDRESS       0x3C                    // 0X3C+SA0 - 0x3C or 0x3D
#define LCDADDRESS        0x27                    // some LCD displays maybe at 0x3F, use I2Cscanner to find the correct address

#define BTRX              13
#define BTTX              12
#define CH1TEMP           6                       // channel 1 and 2 temperature probe on pin 6 and 7, use 4.7k pullup
#define CH2TEMP           7                       //
#define CH3TEMP           8                       // channel 3 temperature probe on pin 8, use 4.7k pullup
#define FANMOTOR          5                       // pwm output controls 12vdc cooling fan
#define RLED              A3
#define GLED              A2
#define BLED              A1
#define FANTEMP           2                       // define fan temperature sensor, Fritzing PCB rev02 only
#define DHTDATA           4                       // DHT11/21 sensor connected to pin 4
#define CH1DEW            9                       // channel 1 dew strap pwm output
#define CH2DEW            10                      // channel 2 dew strap pwm output
#define CH3DEW            3                       // channel 3 - can shadow channel 1 or 2 or NONE (disabled)
#define OLED_SDA          A4                      // connected to SDA pin on OLED
#define OLED_SCL          A5                      // connected to SCL pin on OLED
#define TOGGLESWPIN       A0                      // Toggle switches wired to A0 via resistor divider network

#define MAXCOMMAND        15                      // : + 2 + 10 + # = 14
#define MAXPROBES         4                       // 9, 10, 11, or 12 bits, corresponding to increments of 0.5°C, 0.25°C, 0.125°C, and 0.0625°C, respectively
#define TEMP_PRECISION    10                      // Set the DS18B20s precision, 10bit =0.25degrees, 12 = 0.06degrees 
#define EEPROMSIZE        1024                    // ATMEGA328P 1024 EEPROM - Nano v3
#define NORMAL            1                       // mode of operation, changed by PB switches PB1 and PB2
#define OVERRIDE          2                       // or serial commands n and 1, 2
#define AMBIENT           1                       // constants for tracking mode algorithm, track ambient
#define DEWPOINT          2                       // track dewpoint
#define HALFWAY           3                       // track half way between dew point and ambient
#define OFFSETPOSLIMIT    3                       // these are limits for the offset applied to 
#define OFFSETNEGLIMIT    -4                      // the temp calc routine for pwr to the dew straps
#define CELSIUS           1                       // these used to determine display temperature values in C
#define FAHRENHEIT        2                       // or F
#define BUTTONDELAY       1000                    // Time between switch override checks
#define BUF_SIZE          24                      // Size of buffer for OLED
#define TEMPUPDATES       1000                    // Time in milliseconds of temperature updates
#define MAXPAGETIME       5000
#define MINPAGETIME       2000

// Power percentage levels
#define POWER_0           0
#define POWER_10          10
#define POWER_20          20
#define POWER_50          50
#define POWER_75          75
#define POWER_100         100

// 
#define str0              "0"
#define str50             "5"
#define str75             "7"
#define str100            "F"
#define celstr            "C"       // Celsius
#define fstr              "F"       // fahrenheit
#define astr              "A"       // Ambient
#define dstr              "D"       // dewpoint
#define mstr              "M"       // midpoint
#define offmsgstr         "OFF"
#define onmsgstr          "ON"
#define ch1msgstr         "CH1"
#define ch2msgstr         "CH2"
#define manstr            "MAN"     // manual
#define probestr          "PROB"    // probe
#define plusstr           "+"
#define minusstr          "-"
#define probesequals      "PROBES = "
#define yesstr            "Y"
#define nostr             "N"
#define PCBprobeequalsstr "PCB PROBE = "


#endif
