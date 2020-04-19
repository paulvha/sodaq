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
 *   -requires Adafruit BME280 Library: https://github.com/adafruit/Adafruit_BME280_Library
 *   -requires Adafruit Unified Sensor: https://github.com/adafruit/Adafruit_Sensor
 *
 *  =========================  Highlevel description ================================
 *
 *  This example sketch will connect to an SPS30 and BME280 for getting data and
 *  display the available data. It will also connect with a SODAQ SARA to AllThingsTalk
 *  and display the SPS30-id, Mass1, Mass2, Mass10, Humidity, temperature and pressure values 
 */  
 // ************* for detailed setup see Example2.odt in this folder ******************
 
/* ============================ HARDWARE CONNECTION ==================================
 *  Successfully tested on SODAQ SARA/AFF
 *  Serial and I2C
 *  
 *  SPS30 pin     BME280         SODAQ
 *  1 VCC --------- 5V -------   5V
 *  2 RX ----------------------- 1 TX 
 *  3 TX ----------------------- 0 RX 
 *  4 Select------------ --------     NOT CONNECTED (Select Serial)
 *  5 GND --------- GND--------   GND
 *                  SCK -------   SCL (next to SCL/ARF NOT A5 / SCL1)
 *                  SDI -------   SDA (next to ARF     NOT A4 / SDA1)
 *  
 *  SELECT SP30_COMMS SERIALPORT
 *-----------------------------------------------------------------------------------
 *  I2C ONLY   
 *  As documented in the datasheet, make sure to use external 10K pull-up resistor on
 *  both the SDA and SCL lines. Otherwise the communication with the sensor will fail random.
 *  When connecting as indicated below TO SDA AND SCL those pull-up resistors are already 
 *  on the SODAQ board.
 *
 *  Successfully tested on SODAQ SARA/AFF
 *
 *  SPS30 pin    BME280       SODAQ
 *  1 VCC -------- 5V ------- 5V
 *  2 SDA -------- SDI -------SDA (next to ARF     NOT A4 / SDA1)
 *  3 SCL -------- SCK -------SCL (next to SCL/ARF NOT A5/ SCL1)
 *  4 Select ----- GND -------GND(select I2c)
 *  5 GND ------------------- GND
 *
 *  ================================= PARAMETERS =====================================
 *
 *  From line 85 there are configuration parameters for the program.
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
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
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
//*        BME280 constants                                     **     
//****************************************************************
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME280_ADDRESS  0x77        // some are by default 0x76

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
Adafruit_BME280 bme; // I2C

//********************** GLOBAL VARIABLES ************************

// contains pointers to driver on / off routines
static Sodaq_SARA_R4XX_OnOff saraR4xxOnOff;   

// capture data for sending
float TotalMassPM1  = 0;
float TotalMassPM2  = 0;
float TotalMassPM10 = 0;
uint32_t SampleCnt  = 0;
String SPS_id ;               // SPS30 ID to include 

// BME280 data
float b_temperature = 0;
float b_pressure = 0;
float b_altitude = 0;
float b_humidity = 0;

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
    
    ReadBME280();
    
    setLight(GREEN);   // indicate sending
    
    String deviceId = DEVICE_ID;    // defined in keys.h
    String token = DEVICE_TOKEN;
    
    // create JSON with values
    String value =  "{\"ID\":{\"value\":\"" + String(SPS_id) + "\"}";
           value += ",\"M1\":{\"value\":" + String(TotalMassPM1 / SampleCnt) +"}";
           value += ",\"M2\":{\"value\":" + String(TotalMassPM2 / SampleCnt) +"}";
           value += ",\"M10\":{\"value\":" + String(TotalMassPM10 / SampleCnt) +"}";
           value += ",\"TEMP\":{\"value\":" + String(b_temperature) +"}";
           value += ",\"HUM\":{\"value\":" + String(b_humidity) +"}";
           value += ",\"PRES\":{\"value\":" + String(b_pressure) +"}}";

    String reading = deviceId + '\n' + token + '\n' + value;

    uint8_t rsize = reading.length();
    int lengthSent = r4x.socketSend(socketID, ALLTHINGSTALK_IP, 8891, (uint8_t*)reading.c_str(), rsize);
    r4x.socketClose(socketID);

    // only reset if succesfull
    if (rsize == lengthSent) {
      TotalMassPM1 = 0; TotalMassPM2 = 0; TotalMassPM10 = 0; SampleCnt = 0;
      startMillis = millis(); 
      setLight(OFF);
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
      serialTrigger((char *) "SPS30-SODAQ-Example2: reading with BME280. press <enter> to start");
  
  InitLed();

  setLight(GREEN);       InitBME280();

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

//****************************************************************
//*            read and display device info                     **
//****************************************************************
void GetDeviceInfo()
{
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

//****************************************************************
//**        read and display all SPS30 and BME280 values        **     
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
    DEBUG_STREAM.print(F("===================================================================== SPS30 ========================================================================================="));
    DEBUG_STREAM.println(F("\t   =============== BME280 ================"));
    
    DEBUG_STREAM.print(F("----------------------------Mass -----------------------------    -------------------------------- Number --------------------------------------       --Partsize --"));
    DEBUG_STREAM.println(F("\t   Press\t  Humidity\tTemperature"));
    
    DEBUG_STREAM.print(F("                     Concentration [μg/m3]                                                 Concentration [#/cm3]                                           [μm]  "));
    DEBUG_STREAM.println(F("\t   [Hpa]\t    [%]\t\t    *C"));
    DEBUG_STREAM.println(F(" PM1.0             PM2.5           PM4.0           PM10             PM0.5           PM1.0           PM2.5           PM4.0              PM10               Typical"));
    
    header = false;
  }
  
  ReadBME280();

  print_aligned((double) val.MassPM1, 8, 5);   print_aligned((double) val.MassPM2, 8, 5);  print_aligned((double) val.MassPM4, 8, 5);
  print_aligned((double) val.MassPM10, 8, 5);  print_aligned((double) val.NumPM0, 9, 5);   print_aligned((double) val.NumPM1, 9, 5);
  print_aligned((double) val.NumPM2, 9, 5);    print_aligned((double) val.NumPM4, 9, 5);   print_aligned((double) val.NumPM10, 15, 5);
  print_aligned((double) val.PartSize, 7, 5);  print_aligned((double) b_pressure, 7, 2);   print_aligned((double) b_humidity, 7, 2); 
  print_aligned((double) b_temperature, 7, 2);  DEBUG_STREAM.print(F("\n"));

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
//**                    BME280 FUNCTIONS                        **
//****************************************************************
void InitBME280()
{
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  
  if (! bme.begin(BME280_ADDRESS)) {
    DEBUG_STREAM.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    DEBUG_STREAM.print("SensorID was: 0x"); DEBUG_STREAM.println(bme.sensorID(),16);
    DEBUG_STREAM.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    DEBUG_STREAM.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    DEBUG_STREAM.print("        ID of 0x60 represents a BME 280.\n");
    DEBUG_STREAM.print("        ID of 0x61 represents a BME 680.\n");
    Errorloop((char *) "FAILED TO CONNECT TO BME280", 0);
  }
  
  DEBUG_STREAM.println("BME280 Detected");
}

void ReadBME280()
{
  b_temperature = bme.readTemperature();
  b_pressure = bme.readPressure() / 100.0F;
  b_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  b_humidity = bme.readHumidity();

  if(SKETCH_DEBUG) {
    DEBUG_STREAM.print("Temperature = "); DEBUG_STREAM.print(b_temperature);
    DEBUG_STREAM.println(" *C");

    DEBUG_STREAM.print("Pressure = "); DEBUG_STREAM.print(b_pressure);
    DEBUG_STREAM.println(" hPa");

    DEBUG_STREAM.print("Approx. Altitude = ");  DEBUG_STREAM.print(b_altitude);
    DEBUG_STREAM.println(" m");

    DEBUG_STREAM.print("Humidity = ");  DEBUG_STREAM.print(b_humidity);
    DEBUG_STREAM.println(" %\n");
  }
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
