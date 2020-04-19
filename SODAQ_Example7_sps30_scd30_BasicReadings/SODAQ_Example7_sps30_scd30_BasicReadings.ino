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
 *   -requires at least SPS30 driver version 1.3.10 https://github.com/paulvha/sps30
 *   -requires the latest SCD30 library:  https://github.com/paulvha/scd30
 *
 *  =========================  Highlevel description ================================
 *
 *  This basic reading example sketch will connect to an SPS30 & SCD30 for getting data and
 *  display the available data. It will also connect with a SODAQ SARA to AllThingsTalk
 *  and display the SPS30-id, Mass1, Mass2, Mass10, Humidity, CO2 and Temperature values
 */  
 // ************* for detailed setup see Example7.odt in this folder ****************** 
/* 
 *  ============================ HARDWARE CONNECTION ==================================
 *  
 *  Successfully tested on SODAQ SARA/AFF
 *  Serial and I2C
 *  
 *  SPS30 pin     SCD30         SODAQ
 *  1 VCC -------- 1 VDD -------   5V
 *  2 RX ----------------------- 1 TX 
 *  3 TX ----------------------- 0 RX 
 *  4 Select------------ --------     NOT CONNECTED (Select Serial)
 *  5 GND ---------2 GND--------   GND
 *                 3 SCL -------   SCL (next to SCL/ARF NOT A5 / SCL1)
 *                 4 SDA -------   SDA (next to ARF     NOT A4 / SDA1)
 *                 5 RDY -------      NOT CONNECTED
 *                 6 PWM -------      NOT CONNECTED
 *                 7 SEL -------      NOT CONNECTED
 *  
 *  SELECT SP30_COMMS SERIALPORT
 *-----------------------------------------------------------------------------------
 *  I2C ONLY 
 *  As documented in the datasheet, make sure to use external min 4K7 to 10K pull-up 
 *  resistor on both the SDA and SCL lines. Otherwise the communication with the SPS30 
 *  sensor will fail random. When connecting as indicated below to SDA AND SCL those 
 *  pull-up resistors are already on the SODAQ board.
 *  
 *  SPS30 pin     SCD30         SODAQ
 *  1 VCC -------- 1 VDD ------- 5V
 *  2 SDA -------- 4 SDA --------SDA (next to ARF     NOT A4 / SDA1)
 *  3 SCL -------- 3 SCL --------SCL (next to SCL/ARF NOT A5 / SCL1)
 *  4 Select ----- 2 GND --------GND (SPS30 select I2c)
 *  5 GND ---------------------- GND
 *                 5 RDY -------      NOT CONNECTED
 *                 6 PWM -------      NOT CONNECTED
 *                 7 SEL -------      NOT CONNECTED
 *
 *  SELECT SP30_COMMS I2C_COMMS
 *          
 *  ================================= PARAMETERS =====================================
 *
 *  From line 90 there are configuration parameters for the program.
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
#include "sps30.h"
#include "keys.h"
#include <avr/dtostrf.h>
#include "paulvha_SCD30.h"

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

//*****************************************************************
//* Define debug messages                                        **     
//* 0 : no messages                                              **     
//* 1 : request sending and receiving                            **     
//* 2 : 1 + show protocol errors   (SPS30 only)                  **
//*****************************************************************
#define SPS30_DEBUG 0
#define SCD30_DEBUG 0     // see NOTE 1
#define SODAQ_DEBUG 0
#define SKETCH_DEBUG 0

/* NOTE 1: requires a change in the file paulvha_scd30_library/src/printf.h
 * change line 29 :  #define _Stream_Obj_                Serial
 * to                #define _Stream_Obj_                SerialUSB
 */
 
//////////////////////////////////////////////////////////////
/* Define display in Fahrenheit or Celsius
 *  1 = Celsius
 *  0 = Fahrenheit */
#define TEMP_TYPE 1

//*****************************************************************
//*        measurement constants                                 **     
//*****************************************************************
#define MEASUREMENTPERIOD  60000     // time between sending update to AllTHingSTalk (ms) (60000 = 60 sec)
#define MEASUREINTERVAL    5000      // delay between measurement in mS (5000 = 5 seconds)

///////////////////////////////////////////////////////////////
/////////// NO CHANGES BEYOND THIS POINT NEEDED ///////////////
///////////////////////////////////////////////////////////////

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

//*********************** CONSTRUCTORS ****************************
SPS30 sps30;
Sodaq_R4X r4x;
SCD30 airSensor;

//********************** GLOBAL VARIABLES *************************

// contains pointers to driver on / off routines
static Sodaq_SARA_R4XX_OnOff saraR4xxOnOff;   

// capture data for sending
float TotalMassPM1  = 0;
float TotalMassPM2  = 0;
float TotalMassPM10 = 0;
uint32_t SampleCnt  = 0;
String SPS_id;     // SPS30 ID, will be set to last 4 digits serial number

float SCD_hum = 0;
float SCD_tmp = 0;
float SCD_TotalCO2 = 0;

// status
bool SCD30_detected = false;

unsigned long startMillis;   // Used to keep track of send interval
bool header = true;          // display header

//*****************************************************************
//**                      SEND MESSAGE                           **
//*****************************************************************
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
    
    setLight(GREEN);   // indicate sending
    
    String deviceId = DEVICE_ID;    // defined in keys.h
    String token = DEVICE_TOKEN;
    
    if (SampleCnt == 0) SampleCnt = 1; 
    
    // create JSON values
    String value =  "{\"ID\":{\"value\":\"" + String(SPS_id) + "\"}";
           value += ",\"M1\":{\"value\":" + String(TotalMassPM1 / SampleCnt) +"}";
           value += ",\"M2\":{\"value\":" + String(TotalMassPM2 / SampleCnt) +"}";
           value += ",\"M10\":{\"value\":" + String(TotalMassPM10 / SampleCnt) +"}";
           value += ",\"CO2\":{\"value\":" + String(SCD_TotalCO2 / SampleCnt) +"}";
           value += ",\"HUM\":{\"value\":" + String(SCD_hum) +"}";
           value += ",\"TEMP\":{\"value\":" + String(SCD_tmp) +"}}";

    String reading = deviceId + '\n' + token + '\n' + value;

    uint8_t rsize = reading.length();
    int lengthSent = r4x.socketSend(socketID, ALLTHINGSTALK_IP, 8891, (uint8_t*)reading.c_str(), rsize);
    r4x.socketClose(socketID);

    // only reset if succesfull
    if (rsize == lengthSent) {
      SCD_TotalCO2 = TotalMassPM1 = TotalMassPM2 = TotalMassPM10 = 0; SampleCnt = 0; 
      startMillis = millis(); 
      setLight(OFF);
    }
    else
      setLight(RED);    // indicate error
    
    if (SKETCH_DEBUG)  {
      DEBUG_STREAM.println("data sent: "); DEBUG_STREAM.println(reading); 
      DEBUG_STREAM.print("Length buffer vs sent:"); DEBUG_STREAM.print(rsize); DEBUG_STREAM.print(",");
      DEBUG_STREAM.println(lengthSent); DEBUG_STREAM.println();
    }
}

//*************************************************************
//**                  SETUP FUNCTIONS                        **
//*************************************************************
void setup() {
  
  sodaq_wdt_safe_delay(STARTUP_DELAY);
   
  DEBUG_STREAM.begin(DEBUG_STREAM_BAUD);
 
  if (SKETCH_DEBUG) serialTrigger((char *) "SPS30-SODAQ-Example7: Reading with SCD30. press <enter> to start");

  InitLed(); 
  
  setLight(YELLOW);     InitSpS30();

  setLight(GREEN);      InitSCD30();
  
  setLight(BLUE);       InitSodaq();

  setLight(OFF);

  // start measurement
  if (! sps30.start() )   Errorloop((char *) "Could NOT start measurement", 0);
    
  DEBUG_STREAM.println(F("Measurement started"));

  startMillis = millis();           // Saves the initial millis value 
}

//*************************************************************
//**                   LOOP FUNCTIONS                        **
//*************************************************************
void loop() {
  static uint8_t StillAlive = 0;
  unsigned long ElapseCnt, Interval = MEASUREINTERVAL;
  static bool SPS30_idle = false;

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
    
    if (SKETCH_DEBUG || SPS30_DEBUG || SODAQ_DEBUG || SCD30_DEBUG)
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
    if (Interval > 1000)  Interval -= 1000;   // because we did wait to blink..
  }
  
  read_all(); 
  
  // stop fan and measurement if interval more than 60 seconds
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

//***********************************************************
//*            read and display device info                **
//***********************************************************
void GetDeviceInfo() {
  char buf[32];
  uint8_t ret;

  //try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  
  if (ret == ERR_OK) {
    DEBUG_STREAM.print(F("Serial number : "));
    
    if(strlen(buf) > 0) {
      DEBUG_STREAM.println(buf);
      
      // use the last 4 digits for SPS_id
      for (uint8_t i = 12; i < 16; i++) SPS_id += buf[i];
    }
    else DEBUG_STREAM.println(F("not available"));
  }
  else
    ErrtoMess((char *) "could not get serial number: ", ret);
}

//*********************************************************
//**        read and display all SPS30 / SCD30 values    **     
//*********************************************************
bool read_all() {
  uint8_t ret, error_cnt = 0;
  struct sps_values val;
  float SCD_co2 = 0;

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
    
    DEBUG_STREAM.print(F("===================================================================== SPS30 ========================================================================================="));
    if(SCD30_detected) DEBUG_STREAM.print(F("\t\t========= SCD30 =========="));
    DEBUG_STREAM.print(F("\n----------------------------Mass -----------------------------    -------------------------------- Number --------------------------------------       --Partsize --"));
    
    if(SCD30_detected) DEBUG_STREAM.print(F("\t\tCO2   Humidity Temperature"));
    
    DEBUG_STREAM.print(F("\n                     Concentration [μg/m3]                                                 Concentration [#/cm3]                                           [μm]  "));
    
    if(SCD30_detected) {
      DEBUG_STREAM.print(F("\t\t[ppm]    [%]      "));

      if (TEMP_TYPE) DEBUG_STREAM.print(F("[*C]"));
      else DEBUG_STREAM.print(F("[*F]"));
    }   
    DEBUG_STREAM.println(F("\n PM1.0             PM2.5           PM4.0           PM10             PM0.5           PM1.0           PM2.5           PM4.0              PM10               Typical"));
    header = false;
  }

  print_aligned((double) val.MassPM1, 8, 5);   print_aligned((double) val.MassPM2, 8, 5);  print_aligned((double) val.MassPM4, 8, 5);
  print_aligned((double) val.MassPM10, 8, 5);  print_aligned((double) val.NumPM0, 9, 5);   print_aligned((double) val.NumPM1, 9, 5);
  print_aligned((double) val.NumPM2, 9, 5);    print_aligned((double) val.NumPM4, 9, 5);   print_aligned((double) val.NumPM10, 15, 5);
  print_aligned((double) val.PartSize, 7, 5);

  if(SCD30_detected) {
    
    SCD_co2 = airSensor.getCO2();
    SCD_TotalCO2 += SCD_co2;    
    
    if (TEMP_TYPE) SCD_tmp = airSensor.getTemperature();
    else SCD_tmp =  airSensor.getTemperatureF();
    
    SCD_hum = airSensor.getHumidity();
    
    DEBUG_STREAM.print(F("\t  ")); DEBUG_STREAM.print(SCD_co2,0); DEBUG_STREAM.print(F("\t "));
    DEBUG_STREAM.print(SCD_hum, 1); DEBUG_STREAM.print(F("\t  ")); DEBUG_STREAM.print(SCD_tmp, 2);
  }
  
  DEBUG_STREAM.print(F("\n"));
  
  // collect data for AllThingsTalk
  TotalMassPM1 += val.MassPM1;
  TotalMassPM2 += val.MassPM2;
  TotalMassPM10 += val.MassPM10;
  SampleCnt++;
  
  return(true);
}

//*****************************************************************
//* @brief will print nice aligned columns                       **     
//*                                                              **     
//* @param val   : value to print                                **     
//* @param width : total width of value including decimal point  **
//* @param prec  : precision after the decimal point             **     
//*****************************************************************
void print_aligned(double val, signed char width, unsigned char prec) {
  char out[25];

  dtostrf(val, width, prec, out);
  DEBUG_STREAM.print(out);
  DEBUG_STREAM.print(F("\t  "));
}

//****************************************************************
//**                   INITIALIZE SPS30                         **
//****************************************************************
void InitSpS30() {
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
//**                   INITIALIZE SCD30                         **
//****************************************************************
void InitSCD30() {

  char buf[30];
   
  // set SCD30
  airSensor.setDebug(SCD30_DEBUG);

  // This will init the wire, but NOT start reading
  if ( ! airSensor.begin(Wire,false) )
    DEBUG_STREAM.println(F("cound not start SCD30"));
  else
  {
    DEBUG_STREAM.print(F("Detected SCD30 "));

    if (airSensor.getSerialNumber(buf))
    {
      DEBUG_STREAM.print(F("Serial number: "));
      DEBUG_STREAM.println(buf);
    }
    else
      DEBUG_STREAM.println(F("could not read serial number"));
  }

  // This will cause readings to occur every two seconds
  if (! airSensor.begin() )
    DEBUG_STREAM.println(F("cound not start SCD30"));
  else
    SCD30_detected = true;
}

//****************************************************************
//**                   INITIALIZE SODAQ                         **
//****************************************************************
void InitSodaq() {
  DEBUG_STREAM.print("Initializing and connecting .. ");
  DEBUG_STREAM.println(provider);

  MODEM_STREAM.begin(r4x.getDefaultBaudrate());   // set to 115200 in r4x.h
  
  if (SODAQ_DEBUG) r4x.setDiag(DEBUG_STREAM);
  r4x.init(&saraR4xxOnOff, MODEM_STREAM);

  if (!r4x.connect(apn, urat))  Errorloop((char *) "FAILED TO CONNECT TO MODEM", 0);
}

//****************************************************************
//*                        LED FUNCTIONS                        **     
//****************************************************************
void InitLed() {
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledBlue, OUTPUT);

  digitalWrite(ledRed, HIGH);
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledBlue, HIGH);
}

void setLight(lightColor color) {
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

//*************************************************************
//*  @brief : continued loop after fatal error               **     
//*  @param mess : message to display                        **     
//*  @param r    : error code                                **     
//*                                                          **     
//*  if r is zero, it will only display the message          **     
//*************************************************************
void Errorloop(char *mess, uint8_t r) {
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

//***********************************************************
//*  @brief :  display error message                       **     
//*  @param mess : message to display                      **     
//*  @param r    : error code                              **     
//***********************************************************
void ErrtoMess(char *mess, uint8_t r) {
  char buf[80];

  DEBUG_STREAM.print(mess);

  sps30.GetErrDescription(r, buf, 80);
  DEBUG_STREAM.println(buf);
}

//***********************************************************
//* @brief : prints repeated message, then waits for enter **
//* to come in from the serial port.                       **     
//***********************************************************
void serialTrigger(char * mess) {
  DEBUG_STREAM.println();

  while (! DEBUG_STREAM.available()) {
    DEBUG_STREAM.println(mess);
    delay(2000);
  }

  while (DEBUG_STREAM.available())
    DEBUG_STREAM.read();
}
