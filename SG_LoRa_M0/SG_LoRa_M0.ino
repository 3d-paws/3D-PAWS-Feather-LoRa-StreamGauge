#define COPYRIGHT "Copyright [2024] [University Corporation for Atmospheric Research]"
#define VERSION_INFO "SGLoRaM0-240626"

/*
 *======================================================================================================================
 * StreamGauge(SG) LoRa
 *   Board Type : Adafruit Feather M0
 *   Description: 
 *   Author: Robert Bubon
 *   Date:   2021-03-25 RJB Initial
 *           2021-12-01 RJB Version 2 Added BMP388 support
 *           2022-08-02 RJB NaN check added to BMX sensors
 *           2022-09-22 RJB Added support for 8 line oled displays
 *                          Now logs on 0,15,30,45 times with in the hour
 *           2024-06-26 RJB Split code up into include files
 *                          Added CONFIG.TXT support
 *                          Moved LoRa configs to CONFIG.TXT
 *                          Moved Serial Console from A4 to D12
 *                          Opdated code to match other devices
 *                          Added support for 5m and 10m distance sensors and reworked the scaling factor calculation
 *                          Added Copyright
 *                          
 * Time Format: 2022:09:19:19:10:00  YYYY:MM:DD:HR:MN:SS
 *                    
 * SEE https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module
 * SEE https://learn.adafruit.com/assets/46254 for Pinouts      
 * SEE https://www.microchip.com/wwwproducts/en/ATsamd21g18
 * SEE https://learn.adafruit.com/adafruit-adalogger-featherwing/pinouts
 * SEE https://www.microchip.com/wwwproducts/en/MCP73831 - Battery Charger
 * 
 * Arduino Low Power
 * SEE: https://www.arduino.cc/en/Reference/ArduinoLowPower
 * SEE https://github.com/arduino-libraries/ArduinoLowPower
 *  
 * Arduino/libraries/AES-master/AES_config.h  Modified to fix below problem.
 *     Arduino/libraries/AES-master/AES_config.h:43:18: fatal error: pgmspace.h: No such file or directory
 *     #include <pgmspace.h>
 *                ^~~~~~~~~~~~
 * diff AES_config.h.org AES_config.h
 *     40c40
 *     <     #if (defined(__AVR__))
 *     ---
 *     >     #if (true || defined(__AVR__))
 *     41a42
 *     >  #define printf_P printf
 *
 * Library of Adafruit_BMP280_Library has been modified to enable Forced Mode
 * ======================================================================================================================
 * Pin Definitions
 * 
 * Board Label   Arduino  Info & Usage                   Grove Shield Connector   
 * ======================================================================================================================
 * RST
 * 3V            3v3 Power
 * ARef
 * GND
 * A0            A0       Not in Use                     Grove A0
 * A1            A1       Not in Use                     Grove A0
 * A2            A2       Not in Use                     Grove A2
 * A3            A3       Distance Gauge                 Grove A2
 * A4            A4       Not in Use                     Grove A4
 * A5            A5       Not in Use                     Grove A4
 * SCK           SCK SPI0 Clock - SD/LoRa
 * MOS           MOSI     Used by SD Card/LoRa           Not on Grove
 * MIS           MISO     Used by SDCard/LoRa            Not on Grove
 * RX0           D0                                      Grove UART
 * TX1           D1                                      Grove UART 
 * io1           DIO1                                    Not on Grove (Particle Pin D9)
   
 * BAT           VBAT Power
 * En            Control - Connect to ground to disable the 3.3v regulator
 * USB           VBUS Power
 * 13            D13      LED                            Not on Grove 
 * 12            D12      Serial Console Enable          Not on Grove
 * 11            D11      Not in Use                     Not on Grove
 * 10            D10      Used by SD Card as CS          Grove D4  (Particle Pin D5)
 * 9             D9/A7    Voltage Battery Pin            Grove D4  (Particle Pin D4)
 * 6             D6       Not in Use                     Grove D2  (Particle Pin D3)
 * 5             D5       Not in Use                     Grove D2  (Particle Pin D2)
 * SCL           D3       i2c Clock                      Grove I2C_1
 * SDA           D2       i2c Data                       Grove I2C_1
 * 
 * Not exposed on headers
 * D8 = LoRa NSS aka Chip Select CS
 * D4 = LoRa Reset
 * D3 = LoRa DIO
 */

#include <SPI.h>
#include <Wire.h>
#include <ArduinoLowPower.h>
#include <SD.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BMP3XX.h>

#include <RH_RF95.h>
#include <AES.h>
#include <RTClib.h>

#define PCF8523_ADDRESS 0x68       // I2C address for PCF8523


/*
 * ======================================================================================================================
 * System Status Bits used for report health of systems - 0 = OK
 * 
 * OFF =   SSB &= ~SSB_PWRON
 * ON =    SSB |= SSB_PWROFF
 * 
 * ======================================================================================================================
 */
#define SSB_PWRON           0x1     // Set at power on, but cleared after first observation
#define SSB_SD              0x2     // Set if SD missing at boot or other SD related issues
#define SSB_RTC             0x4     // Set if RTC missing at boot
#define SSB_OLED            0x8     // Set if OLED missing at boot, but cleared after first observation
#define SSB_N2S             0x10    // Set when Need to Send observations exist
#define SSB_FROM_N2S        0x20    // Set in transmitted N2S observation when finally transmitted
#define SSB_AS5600          0x40    // Set if wind direction sensor AS5600 has issues
#define SSB_BMX_1           0x80    // Set if Barometric Pressure & Altitude Sensor missing
#define SSB_BMX_2           0x100   // Set if Barometric Pressure & Altitude Sensor missing
#define SSB_HTU21DF         0x200   // Set if Humidity & Temp Sensor missing
#define SSB_SI1145          0x400   // Set if UV index & IR & Visible Sensor missing
#define SSB_MCP9808         0x800   // Set if Precision I2C Temperature Sensor missing
#define SSB_LORA            0x1000  // Set if LoRa Radio missing at startup

unsigned int SystemStatusBits = SSB_PWRON; // Set bit 0 for initial value power on. Bit 0 is cleared after first obs
bool JustPoweredOn = true;         // Used to clear SystemStatusBits set during power on device discovery

/*
 * =======================================================================================================================
 *  Globals
 * =======================================================================================================================
 */
char msgbuf[RH_RF95_MAX_MESSAGE_LEN+1];   // 255 - 4(Header) + 1(Null) = 252
char *msgp;                               // Pointer to message text
char Buffer32Bytes[32];                   // General storage
int countdown = 1800;        // Exit calibration mode when reaches 0 - protects against burnt out pin or forgotten jumper
unsigned int SendSensorMsgCount=0;        // Counter for Sensor messages transmitted
unsigned int SendType2MsgCount=0;         // Counter for Powerup and Heartbeat messages transmitted
int  LED_PIN = LED_BUILTIN;               // Built in LED


/*
 * ======================================================================================================================
 *  SD Card
 * ======================================================================================================================
 */
#define SD_ChipSelect 10    // GPIO 10 is Pin 10 on Feather and D5 on Particle Boron Board
// SD;                      // File system object defined by the SD.h include file.
File SD_fp;
char SD_obsdir[] = "/OBS";  // Store our obs in this directory. At Power on, it is created if does not exist
bool SD_exists = false;     // Set to true if SD card found at boot

/*
 * ======================================================================================================================
 *  Local Code Includes - Do not change the order of the below 
 * ======================================================================================================================
 */
#include "QC.h"                   // Quality Control Min and Max Sensor Values on Surface of the Earth
#include "SF.h"                   // Support Functions
#include "OP.h"                   // OutPut support for OLED and Serial Console
#include "CF.h"                   // Configuration File Variables
#include "TM.h"                   // Time Management
#include "LoRa.h"                 // LoRa
#include "Sensors.h"              // I2C Based Sensors
#include "SDC.h"                  // SD Card
#include "Stream.h"               // Stream Gauge
#include "OBS.h"                  // Do Observation Processing
#include "SM.h"                   // Station Monitor


/* 
 *=======================================================================================================================
 * seconds_to_next_obs() - do observations on 0, 15, 30, or 45 minute window
 *=======================================================================================================================
 */
int seconds_to_next_obs() {
  now = rtc.now(); //get the current date-time
  return (900 - (now.unixtime() % 900)); // 900 = 60s * 15m,  The mod operation gives us seconds passed in this 15m window
}



/*
 * =======================================================================================================================
 * setup()
 * =======================================================================================================================
 */
void setup() 
{
  // Put initialization like pinMode and begin functions here.
  pinMode (LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Output_Initialize();
  delay(2000); // Prevents usb driver crash on startup

  Serial_write(COPYRIGHT);
  Output (VERSION_INFO);
  delay (4000);      // Pause so user can see version if not waiting for serial

  // https://forums.adafruit.com/viewtopic.php?f=57&t=174492&p=850337&hilit=RFM95+adalogger#p850337
  // Normally, well-behaved libraries for SPI devices would take care to set CS high when inactive. 
  // But since the SD library does not initialize the radio, pin 8 is left floating.
  // When I need to use the radio again, should I set pin 8 to low?
  // No, the driver will handle it for you. You just have to make sure it is high when not in use.
  
  pinMode(LORA_SS, OUTPUT);
  digitalWrite(LORA_SS, HIGH);
  // pinMode(LORA_SS, INPUT_PULLUP); // required since RFM95W is also on the SPI bus

  // Initialize SD card if we have one.
  SD_initialize();
  
  if (SD_exists && SD.exists(CF_NAME)) {
    SD_ReadConfigFile();
  }
  else {
    sprintf(msgbuf, "CF:NO %s", CF_NAME); Output (msgbuf);
  }
  
  // Read RTC and set system clock if RTC clock valid
  rtc_initialize();

  if (RTC_valid) {
    Output("RTC: Valid");
  }
  else {
    Output("RTC: Not Valid");
  }

  rtc_timestamp();
  sprintf (msgbuf, "%s", timestamp);
  Output(msgbuf);
  delay (2000);

  //=====================
  // Adafruit i2c Sensors
  //=====================
  bmx_initialize();

  lora_initialize();
}

/*
 * =======================================================================================================================
 * loop()
 * =======================================================================================================================
 */
void loop()
{
  // RTC not set, Get Time for User
  if (!RTC_valid) {
    static bool first = true;

    delay (1000);
      
    if (first) {
      if (digitalRead(SCE_PIN) != LOW) {
        Serial.begin(9600);
        SerialConsoleEnabled = true;
      }  
    
      Output("SET RTC ENTER:");
      Output("YYYY:MM:DD:HH:MM:SS");
      first = false;
    }
    
    if (rtc_readserial()) { // check for serial input, validate for rtc, set rtc, report result
      Output("!!!!!!!!!!!!!!!!!!!");
      Output("!!! Press Reset !!!");
      Output("!!!!!!!!!!!!!!!!!!!");

      while (true) {
        delay (1000);
      }
    }
  }

  //Calibration mode, You can also reset the RTC here
  else if (countdown && digitalRead(SCE_PIN) == LOW) { 
    // Every minute, Do observation (don't save to SD) and transmit - So we can test LoRa
    I2C_Check_Sensors();
    
    if ( (countdown%60) == 0) { 
      OBS_Do(false);
    }
    
    if (BMX_1_exists || BMX_2_exists) {
      StationMonitor();
    }
    else {
      float batt = vbat_get();
      sprintf (Buffer32Bytes, "SG:%3d %d.%02d %04X", 
        (int) analogRead(STREAMGAUGE)/4,    // Pins are 12bit resolution (0-4095), values need to be (0-1023)
        (int)batt, (int)(batt*100)%100,
        SystemStatusBits); 
      Output (Buffer32Bytes);
    }
    
    // check for input sting, validate for rtc, set rtc, report result
    if (Serial.available() > 0) {
      rtc_readserial(); // check for serial input, validate for rtc, set rtc, report result
    }
    
    countdown--;
    delay (1000);
  }

  // Normal Operation
  else {
    I2C_Check_Sensors();
    OBS_Do(true);

    // Shutoff System Status Bits related to initialization after we have logged first observation
    JPO_ClearBits();
    
    Output("Going to Sleep");

    rf95.sleep(); // Shutdown LoRa. Note: it turn on when accessed
    
    delay(2000);    
    OLED_sleepDisplay();

    // At this point we need to determine seconds to next 0, 15, 30, or 45 minute window
    LowPower.sleep(seconds_to_next_obs()*1000); // uses milliseconds
 
    OLED_wakeDisplay();   // May need to toggle the Display reset pin.
    delay(2000);
    OLED_ClearDisplayBuffer(); 
    Output("Wakeup");
  }
}
