/************************************************************************************
 *  Copyright (c) January 2019, version 1.0     Paul van Haastrecht
 *
 *  SPS30 Version 1.1 Paul van Haastrecht
 *  - Changed the I2C information / setup.
 *
 *  SPS30 Version 1.1.1 Paul van Haastrecht / March 2020
 *  - Fixed compile errors and warnings.
 *  
 *  Version 2.0 Paul van Haastrecht / April 2020
 *   -Version for Sodaq and AllThingsTalk
 *   -requires the following SODAQ drivers :
 *   https://github.com/SodaqMoja/Sodaq_R4X
 *   https://github.com/SodaqMoja/Sodaq_wdt
 *   -requires at least SPS30 driver version 1.3.10: https://github.com/paulvha/sps30
 * 
 *   For DS18x20 Temperature sensor obtain  https://github.com/PaulStoffregen/OneWire
 *
 *  =========================  Highlevel description ================================
 *
 *  This example sketch will connect to an SPS30 and DS18X20 for getting data and
 *  display the available data. It will also connect with a SODAQ SARA to AllThingsTalk
 *  and display the SPS30-id, Mass1, Mass2, Mass10 and temperature values 
 */  
 // ************* for detailed setup see Example4.odt in this folder ******************
 
/* ============================ HARDWARE CONNECTION ==================================
 *  Successfully tested on SODAQ SARA/AFF
 *  
 *  Serial
 *  
 *  SPS30 pin     SODAQ
 *  1 VCC -------   5V
 *  2 RX -------- 1 TX 
 *  3 TX -------- 0 RX 
 *  4 Select-----     NOT CONNECTED (Select Serial)
 *  5 GND -------   GND
 *  
 *               DS18x20           SODAQ
 *   ------      GND   ------------- GND
 * |-| 4k7 |--   VCC /red ---------  3V3
 * | ------
 * |-----------  data/yellow ------- D4
 * 
 *  You MUST connect a resistor of atleast 4K7 between data and VCC for pull-up.
 * 
 *  SELECT SP30_COMMS SERIALPORT
 *-----------------------------------------------------------------------------------
 *  I2C ONLY   
 *  As documented in the datasheet, make sure to use external 10K pull-up resistor on
 *  both the SDA and SCL lines. Otherwise the communication with the sensor will fail random.
 *  When connecting as indicated below TO SDA AND SCL those pull-up resistors are already 
 *  on the SODAQ board.
 *

 *
 *  SPS30 pin         SODAQ
 *  1 VCC ------------- 5V
 *  2 SDA -------------SDA (next to ARF     NOT A4 / SDA1)
 *  3 SCL -------------SCL (next to SCL/ARF NOT A5/ SCL1)
 *  4 Select ----------GND(select I2c)
 *  5 GND ------------ GND
 *
 *               DS18x20           SODAQ
 *   ------      GND   ------------- GND
 * |-| 4k7 |--   VCC /red ---------  3V3
 * | ------
 * |-----------  data/yellow ------- D4
 * 
 *  You MUST connect a resistor of atleast 4K7 between data and VCC for pull-up.
 *  
 *  SELECT SP30_COMMS I2C_COMMS
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *  WARNING!! WARNING !!WARNING!! WARNING !!WARNING!! WARNING !! WARNING!! WARNING !!
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *  The DS18B20 works with VCC between 3 and 5V. It works fine on 3.3V however if connected to 5V
 *  and you have an Sodaq, you MUST include a level shifter or making a bridge with resistors like below
 *
 *            -------            -------
 *  GND ------| 10K  |-----!---- | 5k6 |------  data/yellow from DS18x20
 *            --------     !     -------
 *                         !
 *                         pin D4 (SODAQ)
 *                         
 *
 *  Code for the DS18x20 is based on the DS18x20 example in the onewire library                       
 *  
 *  ================================= PARAMETERS =====================================
 *
 *  From line 115 there are configuration parameters for the program.
 */
 //###################################################################################
 // !!!!!!!!!!!!!! Also update the attached keys.h file with device information !!!!!!
 //###################################################################################
 /*
 *  ================================ Disclaimer ======================================
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *  ===================================================================================
 *
 *  NO support, delivered as is, have fun, good luck !!
 */
#include <Sodaq_R4X.h>
#include <Sodaq_wdt.h>
#include <OneWire.h>
#include "sps30.h"
#include "keys.h"
#include <avr/dtostrf.h>

//*****************************************************************
//**                SELECT SPS30 connection (see above)          **
//** I2C    -> I2C_COMMS                                         **
//** serial -> SERIALPORT                                        **
//*****************************************************************
#define SP30_COMMS SERIALPORT

//*****************************************************************
//**                SELECT PROVIDER INFORMATION                  **
//*****************************************************************

// NL VODAFONE NB-IoT
 const char* provider = "NL VODAFONE NB-IoT";
 const char* apn = "nb.inetd.gdsp";
 const char* forceOperator = "20404"; // optional - depends on SIM / network
 const char* urat = "8";

// NL VODAFONE LTE-M
//const char* provider = "NL VODAFONE LTE-M";
//const char* apn = "live.vodafone.com";
//const char* forceOperator = "20404"; // optional - depends on SIM / network
//const char* urat = "7";

// NL KPN
//const char* provider = "NL KPN";
//const char* apn = "internet.m2m";
//const char* forceOperator = "20408"; // optional - depends on SIM / network

//****************************************************************
//* Define debug messages                                       **     
//* 0 : no messages                                             **     
//* 1 : messages + sending and receiving                        **     
//* 2 : 1 + show protocol errors   (SPS30 only)                 **
//****************************************************************
#define SPS30_DEBUG  0
#define SODAQ_DEBUG  0
#define SKETCH_DEBUG 0

//****************************************************************
//*        measurement constants                                **     
//****************************************************************
#define MEASUREMENTPERIOD  60000     // time between sending update to AllThingSTalk (ms) (60000 = 60 sec)
#define MEASUREINTERVAL    5000      // delay between measurement in mS (5000 = 5 seconds)

//****************************************************************
//*        DS18x20  configuration                               **     
//****************************************************************

/* To which pin is the data wire of the DS18x20 connected
 */
#define TEMP_PIN 4          // see remark in hardware section in begin sketch

/* Define reading in Fahrenheit or Celsius
 *  1 = Celsius
 *  0 = Fahrenheit */
#define TEMP_TYPE 1

//////////////////////////////////////////////////////////////////
///////////// NO CHANGES BEYOND THIS POINT NEEDED ////////////////
//////////////////////////////////////////////////////////////////

//********************** DEFINITIONS *****************************

#if defined(ARDUINO_SODAQ_AUTONOMO)
/* SODAQ AUTONOMO + SODAQ NB-IoT R41XM Bee */
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial1
#define powerPin BEE_VCC
#define enablePin BEEDTR
#warning "ARDUINO_SODAQ_AUTONOMO selected"

#elif defined(ARDUINO_SODAQ_SARA)
/* SODAQ SARA AFF*/
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial1
#define powerPin SARA_ENABLE
#define enablePin SARA_TX_ENABLE
#warning "ARDUINO_SODAQ_SARA selected"

#elif defined(ARDUINO_SODAQ_SFF)
/* SODAQ SARA SFF*/
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial
#define powerPin SARA_ENABLE
#define enablePin SARA_TX_ENABLE
#warning "ARDUINO_SODAQ_SFF selected"

#else
#error "Please use one of the listed boards or add your board."
#endif

#define DEBUG_STREAM_BAUD 115200
#define STARTUP_DELAY 5000        // WDT

//*********************** CONSTRUCTORS ***************************
SPS30 sps30;
Sodaq_R4X r4x;
OneWire ds(TEMP_PIN);

//********************** GLOBAL VARIABLES ************************

// contains pointers to driver on / off routines
static Sodaq_SARA_R4XX_OnOff saraR4xxOnOff;   

// capture data for sending
float TotalMassPM1  = 0;
float TotalMassPM2  = 0;
float TotalMassPM10 = 0;
uint32_t SampleCnt  = 0;
String SPS_id ;               // SPS30 ID to include 

// store temperature DS18x20 type and address
byte type_s;                  // holds type. 0xf = NOT detected
byte addr[8];
float ds_temperature;         // store measured temperature

unsigned long startMillis;   // Used to keep track of send interval
bool header = true;          // display header

//****************************************************************
//**                      SEND MESSAGE                          **
//****************************************************************
void sendMessageThroughUDP()
{
    if (SKETCH_DEBUG) DEBUG_STREAM.println("\nSending message through UDP");

    int localPort = 16666;
    int socketID = r4x.socketCreate(localPort);

    if (socketID >= 7 || socketID < 0) {
        DEBUG_STREAM.println("Failed to create socket");
        return;
    }

    if (SKETCH_DEBUG) DEBUG_STREAM.println("Created socket!");
    
    ds_temperature = read_Temperature();
    
    setLight(GREEN);   // indicate sending
    
    String deviceId = DEVICE_ID;    // defined in keys.h
    String token = DEVICE_TOKEN;
    
    // create JSON with values
    String value =  "{\"ID\":{\"value\":\"" + String(SPS_id) + "\"}";
           value += ",\"M1\":{\"value\":" + String(TotalMassPM1 / SampleCnt) +"}";
           value += ",\"M2\":{\"value\":" + String(TotalMassPM2 / SampleCnt) +"}";
           value += ",\"M10\":{\"value\":" + String(TotalMassPM10 / SampleCnt) +"}";
           value += ",\"TEMP\":{\"value\":" + String(ds_temperature) +"}}";

    String reading = deviceId + '\n' + token + '\n' + value;

    uint8_t rsize = reading.length();
    int lengthSent = r4x.socketSend(socketID, ALLTHINGSTALK_IP, 8891, (uint8_t*)reading.c_str(), rsize);
    r4x.socketClose(socketID);

    // only reset if succesfull
    if (rsize == lengthSent) {
      TotalMassPM1 = 0; TotalMassPM2 = 0; TotalMassPM10 = 0; SampleCnt = 0;
      startMillis = millis(); setLight(OFF);
    }
    else
      setLight(RED);    // indicate error
    
    if (SKETCH_DEBUG) {
      DEBUG_STREAM.println("data sent: "); DEBUG_STREAM.println(reading); 
      DEBUG_STREAM.print("Length buffer vs sent:"); DEBUG_STREAM.print(rsize); DEBUG_STREAM.print(",");
      DEBUG_STREAM.println(lengthSent); DEBUG_STREAM.println();
    }
}

//****************************************************************
//**                  SETUP FUNCTIONS                           **
//****************************************************************
void setup() {
  
  sodaq_wdt_safe_delay(STARTUP_DELAY);
 
  DEBUG_STREAM.begin(DEBUG_STREAM_BAUD);
  
  if (SKETCH_DEBUG ||SPS30_DEBUG || SODAQ_DEBUG) 
      serialTrigger((char *) "SPS30-SODAQ-Example4: reading with DS18X20. press <enter> to start");
  
  InitLed();

  setLight(GREEN);       InitDs18x20();

  setLight(YELLOW);      InitSpS30();
  
  setLight(BLUE);        InitSodaq();
  
  setLight(OFF);
  
  // start measurement
  if (! sps30.start() )  Errorloop((char *) "Could NOT start measurement", 0);
    
  DEBUG_STREAM.println(F("Measurement started"));

  startMillis = millis();           // Saves the initial millis value 
}

//****************************************************************
//**                   LOOP FUNCTIONS                           **
//****************************************************************
void loop() {
  static uint8_t StillAlive = 0;
  unsigned long ElapseCnt, Interval = MEASUREINTERVAL;
  bool SPS30_idle = false;

  // if stopped as Measurement Interval was above limit, restart first.
  // startup time is less than 8 seconds, 10 has been taken to be save
  // this time was removed from the Measurement Interval earlier.
  if (SPS30_idle) {
    if (! sps30.start() )  Errorloop((char *) "Could NOT start measurement", 0);
    delay(10000);
    SPS30_idle = false;
    Interval = MEASUREINTERVAL; 
  }
  
  ElapseCnt = millis() - startMillis;
  
  if (ElapseCnt >= MEASUREMENTPERIOD) { 
    sendMessageThroughUDP();
    Interval = MEASUREINTERVAL;  
    
    if (SKETCH_DEBUG || SPS30_DEBUG || SODAQ_DEBUG)
       header = true;   // re-print header only if debug messages were shown
  }
  else {
    // if time to go is less than measurement interval
    if (MEASUREMENTPERIOD - ElapseCnt < MEASUREINTERVAL)
        Interval = MEASUREMENTPERIOD - ElapseCnt;
  }
  
  // blink led
  if (++StillAlive > 5) {
    setLight(MAGENTA);
    delay(1000);
    setLight(OFF);
    StillAlive = 0;
  }

  read_all();

  // stop ventilator and measurement if interval more than 60 seconds
  // you can adjust this to your needs, but 8 seconds (10 sec. to be save)
  // is needed to restart from idle mode (datasheet page 2)
  if (Interval >= 60000) {
    if (! sps30.stop() )  Errorloop((char *) "Could NOT stop measurement", 0);
    SPS30_idle = true;
    
    // startup time is less than 8 seconds, 10 has been taken to be save
    // 10 seconds will be applied after restart
    Interval -= 10000;
  }
  
  delay(Interval);
}

//****************************************************************
//*            read and display device info                     **
//****************************************************************
void GetDeviceInfo()
{
  char buf[32];
  uint8_t ret, cnt = 0;
  
  while(cnt < 2) {    // retry 3 times !
    
    //try to read serial number
    ret = sps30.GetSerialNumber(buf, 32);
    
    if (ret == ERR_OK) {
      DEBUG_STREAM.print(F("Serial number : "));
      
      if(strlen(buf) > 0) {
        DEBUG_STREAM.println(buf);
        
        // use the last 4 digits for SPS_id
        for (uint8_t i = 12; i < 16; i++) SPS_id += buf[i];
        return;
      }
      else DEBUG_STREAM.println(F("not available"));
    }
    else {
      ErrtoMess((char *) "could not get serial number: ", ret);
    }
    
    cnt++;
  }
}

//****************************************************************
//**        read and display all SPS30 values                   **     
//****************************************************************
bool read_all()
{
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {
      ret = sps30.GetValues(&val);
  
      // data might not have been ready
      if (ret == ERR_DATALENGTH) {
  
          if (error_cnt++ > 3) {
            ErrtoMess((char *) "Error during reading values: ",ret);
            return(false);
          }
          delay(1000);
      }
  
      // if other error
      else if(ret != ERR_OK) {
        ErrtoMess((char *) "Error during reading values: ",ret);
        return(false);
      }

   } while (ret != ERR_OK);

  // only print header first time
  if (header) {
    DEBUG_STREAM.print(F("----------------------------Mass -----------------------------    -------------------------------- Number --------------------------------------       --Partsize --"));
    if(type_s != 0xf) DEBUG_STREAM.print("\t--Temperature--");
    DEBUG_STREAM.print(F("\n                     Concentration [μg/m3]                                                 Concentration [#/cm3]                                           [μm]  "));
    
    if(type_s != 0xf) {
      if (TEMP_TYPE)  DEBUG_STREAM.print("\t    Celsius");
      else DEBUG_STREAM.print("\t  Fahrenheit");
    }
    
    DEBUG_STREAM.println(F("\n PM1.0             PM2.5           PM4.0           PM10             PM0.5           PM1.0           PM2.5           PM4.0              PM10               Typical"));
    header = false;
  }

  print_aligned((double) val.MassPM1, 8, 5);   print_aligned((double) val.MassPM2, 8, 5);  print_aligned((double) val.MassPM4, 8, 5);
  print_aligned((double) val.MassPM10, 8, 5);  print_aligned((double) val.NumPM0, 9, 5);   print_aligned((double) val.NumPM1, 9, 5);
  print_aligned((double) val.NumPM2, 9, 5);    print_aligned((double) val.NumPM4, 9, 5);   print_aligned((double) val.NumPM10, 15, 5);
  print_aligned((double) val.PartSize, 7, 5);  
     
  if(type_s != 0xf){
    ds_temperature = read_Temperature();
    print_aligned((double) ds_temperature, 9, 2); 
  }
  
  DEBUG_STREAM.print(F("\n"));

  // collect data for AllThingsTalk
  TotalMassPM1 += val.MassPM1;
  TotalMassPM2 += val.MassPM2;
  TotalMassPM10 += val.MassPM10;
  SampleCnt++;
  
  return(true);
}

//****************************************************************
//* @brief will print nice aligned columns                      **     
//*                                                             **     
//* @param val   : value to print                               **     
//* @param width : total width of value including decimal point **
//* @param prec  : precision after the decimal point            **     
//****************************************************************
void print_aligned(double val, signed char width, unsigned char prec)
{
  char out[25];

  dtostrf(val, width, prec, out);
  DEBUG_STREAM.print(out);
  DEBUG_STREAM.print(F("\t  "));
}

//****************************************************************
//**                   INITIALIZE SPS30                         **
//****************************************************************
void InitSpS30()
{
  DEBUG_STREAM.println(F("Trying to connect SPS30"));

  // set driver debug level
  sps30.EnableDebugging(SPS30_DEBUG, SODAQ);

  // Begin communication channel;
  if (! sps30.begin(SP30_COMMS) )
    Errorloop((char *) "Could not initialize SPS30 communication channel.", 0);

  // check for SPS30 connection
  if (! sps30.probe() )
    Errorloop((char *) "Could not probe / connect with SPS30.", 0);
  else
    DEBUG_STREAM.println(F("Detected SPS30"));

  // reset SPS30 connection
  if (! sps30.reset() )
     Errorloop((char *) "Could not reset SPS30.", 0);

  // read device info
  GetDeviceInfo();
}

//****************************************************************
//**                   INITIALIZE SODAQ                         **
//****************************************************************
void InitSodaq()
{
  DEBUG_STREAM.print("Initializing and connecting .. ");
  DEBUG_STREAM.println(provider);

  MODEM_STREAM.begin(r4x.getDefaultBaudrate());   // set to 115200 in r4x.h
  
  if (SODAQ_DEBUG) r4x.setDiag(DEBUG_STREAM);
  r4x.init(&saraR4xxOnOff, MODEM_STREAM);

  if (!r4x.connect(apn, urat))  Errorloop((char *) "FAILED TO CONNECT TO MODEM", 0);
}

//****************************************************************
//**                    DS18X20 FUNCTIONS                        **
//****************************************************************
void InitDs18x20()
{
  type_s = 0xf;            // indicate no sensor

  if (TEMP_PIN == 0) return;  // no sensor pin defined

  DEBUG_STREAM.print(F("Try to detect temperature sensor. "));

  if ( !ds.search(addr)) {
    DEBUG_STREAM.println(F("No temperature sensor detected."));
    return;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println(F(" CRC is not valid!"));
    return;
  }

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      DEBUG_STREAM.println(F("  Chip = DS18S20"));  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      DEBUG_STREAM.println(F("  Chip = DS18B20"));
      type_s = 0;
      break;
    case 0x22:
      DEBUG_STREAM.println(F("  Chip = DS1822"));
      type_s = 0;
      break;
    default:
      DEBUG_STREAM.println(F("Device is not a DS18x20 family device."));
      return;
  }
}

/**
 * @brief : if DS18x20 sensor detected try to read it
 */
float read_Temperature()
{
  byte i;
  byte present = 0;
  byte data[12];
  float celsius, fahrenheit;

  if (type_s == 0xf) return(0);

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);          // start conversion, with parasite power on at the end

  delay(1000);                // maybe 750ms is enough, maybe not

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);             // Read Scratchpad

  for ( i = 0; i < 9; i++) {  // we need 9 bytes
    data[i] = ds.read();
  }

  // Convert the data to actual temperature because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3;            // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;      // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;

  if (TEMP_TYPE)  return(celsius);
  else return(fahrenheit);
}


//****************************************************************
//*                        LED FUNCTIONS                        **     
//****************************************************************
void InitLed()
{
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledBlue, OUTPUT);

  digitalWrite(ledRed, HIGH);
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledBlue, HIGH);
}

void setLight(lightColor color)
{
  digitalWrite(ledRed, HIGH);
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledBlue, HIGH);

  switch (color)
  {
    case RED:
      digitalWrite(ledRed, LOW);
      break;

    case GREEN:
      digitalWrite(ledGreen, LOW);
      break;

    case BLUE:
      digitalWrite(ledBlue, LOW);
      break;  

    case YELLOW:
      digitalWrite(ledRed, LOW);
      digitalWrite(ledGreen, LOW);
      break;

    case MAGENTA:
      digitalWrite(ledRed, LOW);
      digitalWrite(ledBlue, LOW);
      break;

    case CYAN:
      digitalWrite(ledGreen, LOW);
      digitalWrite(ledBlue, LOW);
      break;

    case WHITE:
      digitalWrite(ledRed, LOW);
      digitalWrite(ledGreen, LOW);
      digitalWrite(ledBlue, LOW);
      break;
      
    default:
      break;
  }
}

//****************************************************************
//*  @brief : continued loop after fatal error                  **     
//*  @param mess : message to display                           **     
//*  @param r    : error code                                   **     
//*                                                             **     
//*  if r is zero, it will only display the message             **     
//****************************************************************
void Errorloop(char *mess, uint8_t r)
{
  if (r) ErrtoMess(mess, r);
  else DEBUG_STREAM.println(mess);
  DEBUG_STREAM.println(F("Program on hold"));
  
  while(1) {
    setLight(RED);
    delay(1000);
    setLight(OFF);
    delay(1000);
  }
}

//****************************************************************
//*  @brief :  display error message                            **     
//*  @param mess : message to display                           **     
//*  @param r    : error code                                   **     
//****************************************************************
void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];

  DEBUG_STREAM.print(mess);

  sps30.GetErrDescription(r, buf, 80);
  DEBUG_STREAM.println(buf);
}

//****************************************************************
//* @brief : prints repeated message, then waits for enter      **
//* to come in from the serial port.                            **     
//****************************************************************
void serialTrigger(char * mess)
{
  DEBUG_STREAM.println();

  while (! DEBUG_STREAM.available()) {
    DEBUG_STREAM.println(mess);
    delay(2000);
  }

  while (DEBUG_STREAM.available())
    DEBUG_STREAM.read();
}
