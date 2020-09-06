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
 *   -requires latest SPS30 driver https://github.com/paulvha/sps30
 *
 * Version 3.0 Paul van Haastrecht / September 2020
 *  - added T-Mobile for Sodaq
 *  - added ABCL and CBOR data format
 *  - if CBOR is enabled https://github.com/telecombretagne/YACL is needed
 *  
 *  =========================  Highlevel description ================================
 *
 *  This basic value example sketch will connect to an SPS30 for getting data and
 *  display the available data. 
 *  In case of vodafone :
 *  It will also connect with a SODAQ SARA to AllThingsTalk
 *  and display the SPS30-id, Mass1, Mass2, Mass10 values 
 *  
 *  In case of T-mobile :
 *  It will connect with a SODAQ to CDG platform from T-Mobile
 *  Data can be forwarded to AllTHingsTalk following instructions 
 *  from https://docs.iotcreators.com/docs/integration-with-allthingstalk
 */  
 // ************* for detailed setup see Example1.odt in this folder ****************** 
/* 
 *  ============================ HARDWARE CONNECTION ==================================
 *  Successfully tested on SODAQ SARA/AFF
 *  
 *  Serial
 *  
 *  SPS30 pin     SODAQ
 *  1 VCC -------   5V
 *  2 RX -------- 1 TX 
 *  3 TX -------- 0 RX 
 *  4 Select-----   NOT CONNECTED (Select Serial)
 *  5 GND -------   GND
 *  
 *  SELECT SP30_COMMS SERIALPORT
 *-----------------------------------------------------------------------------------
 *  I2C ONLY   
 *  As documented in the datasheet, make sure to use external 10K pull-up resistor on
 *  both the SDA and SCL lines. Otherwise the communication with the sensor will fail random.
 *  When connecting as indicated below TO SDA AND SCL those pull-up resistors are already 
 *  on the SODAQ board.
 *
 *  SPS30 pin     SODAQ
 *  1 VCC -------- 5V
 *  2 SDA -------- SDA (next to ARF     NOT A4 / SDA1)
 *  3 SCL -------- SCL (next to SDA/ARF NOT A5/ SCL1)
 *  4 Select ----- GND (select I2c)
 *  5 GND -------- GND
 *  
 *  SELECT SP30_COMMS I2C_COMMS
 *  ================================= PARAMETERS =====================================
 *
 *  From line 125 there are configuration parameters for the program.
 */
 //###################################################################################
 // !!!!!!!!!!!!!! for Vodafone connection                                      !!!!!
 // !!!!!!!!!!!!!! Also update the attached keys.h file with device information !!!!!!
 //###################################################################################
 /*
 *  ================================ Disclaimer ======================================
 *  
 * Parts of the code have been taken from public available SODAQ example programmes :
 * 
 * Copyright (c) 2019, SODAQ
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *  
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *  ===================================================================================
 *
 * NO support, delivered as is, have fun, good luck !!  
 */
#include "Sodaq_pvh.h"
#include "sps30.h"
#include "keys.h"
#include <avr/dtostrf.h>

//*****************************************************************
//**    include Concise Binary Object Representation (CBOR)      **
//**    requires the YACL library to be downloaded               **
//**    https://github.com/telecombretagne/YACL                  **
//**    uncomment define to include                              **
//*****************************************************************
//#define INCLUDE_CBOR 1

//*****************************************************************
//**                SELECT SPS30 connection (see above)          **
//** I2C    -> I2C_COMMS                                         **
//** serial -> SERIALPORT                                        **
//*****************************************************************
#define SP30_COMMS SERIALPORT

//*****************************************************************
//**                SELECT ID length                             **
//** default is last 4 digits from SPS30 serialnumber            **
//*****************************************************************
#define SPSID 4

//*****************************************************************
//**                SELECT PROVIDER INFORMATION                  **
//*****************************************************************

// NL VODAFONE NB-IoT 
//*
 const char* provider = "NL VODAFONE NB-IoT";
 const char* apn = "nb.inetd.gdsp";
 const char* forceOperator = "20404"; 
 const char* urat = "8";
 const char* API_ENDPOINT = "api.allthingstalk.io";
 const uint16_t API_PORT = 8891;

Select_Operator Operator  = S_VODAFONE;     // use Jason format
//Select_Operator Operator  = S_VODAFONE_A;   // use ABCL format
//Select_Operator Operator  = S_VODAFONE_C;   // use CBOR format
//*/
// NL VODAFONE LTE-M (not tested)
/*
const char* provider = "NL VODAFONE LTE-M";
const char* apn = "live.vodafone.com";
const char* forceOperator = "20404"; 
const char* urat = "7";
const uint16_t API_PORT = 8891;

Select_Operator Operator  = S_VODAFONE;     // use Jason format
//Select_Operator Operator  = S_VODAFONE_A;   // use ABCL format
//Select_Operator Operator  = S_VODAFONE_C;   // use CBOR format
*/

// NL KPN (not tested)
/*
const char* provider = "NL KPN";
const char* apn = "internet.m2m";
const char* forceOperator = "20408"; 
const uint16_t API_PORT = 8891;
Select_Operator Operator  = S_KPN;
Select_Operator Operator  = S_KPN;     // use Jason format
//Select_Operator Operator  = S_KPN_A;   // use ABCL format
//Select_Operator Operator  = S_KPN_C;  // use CBOR format
*/

// NL T-Mobile 
/*
 const char* provider = "NL T-MOBILE NB-IoT";
 const char* apn = "cdp.iot.t-mobile.nl";
 const char* forceOperator = "20416"; 
 const char* urat = "8";
 const char* API_ENDPOINT = "172.27.131.100";
 const uint16_t API_PORT = 15683;

Select_Operator Operator  = S_TMOBILE;     // use Jason format
//Select_Operator Operator  = S_TMOBILE_A;   // use ABCL format
//Select_Operator Operator  = S_TMOBILE_C;     // use CBOR format
*/

//*****************************************************************
//* Define debug messages                                        **     
//* 0 : no messages                                              **     
//* 1 : request sending and receiving                            **     
//* 2 : 1 + show protocol errors      (SPS30 only)               **
//*****************************************************************
#define SPS30_DEBUG 0
#define SODAQ_DEBUG 0
#define SKETCH_DEBUG 0

//*****************************************************************
//*        measurement constants                                 **     
//*****************************************************************
#define MEASUREMENTPERIOD  60000     // time between sending update to AllThingsTalk (ms) (60000 = 60 sec)
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

//*********************** include CBOR ***************************
#ifdef INCLUDE_CBOR
#include "YACL.h"
#endif

//*********************** CONSTRUCTORS ****************************
SPS30 sps30;
Sodaq_R4X r4x;

//********************** GLOBAL VARIABLES *************************

// contains pointers to driver on / off routines
static Sodaq_SARA_R4XX_OnOff saraR4xxOnOff;   

float TotalMassPM1  = 0;    // capture data for sending
float TotalMassPM2  = 0;
float TotalMassPM10 = 0;
uint32_t SampleCnt  = 0;

char SPS_id[SPSID +1];       // last digits (default 4) serial number + 0x0 
uint8_t SendBuf[100];        // will contain the UDP package to be sent
String TempBuf;              // temp space
uint8_t cnt;                 // number of bytes in SendbBuf

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
    setLight(RED);    // indicate error
    return;
  }

  if (SKETCH_DEBUG) DEBUG_STREAM.println("Created socket!");
  
  setLight(GREEN);   // indicate sending

  //**************** create payload ***********************
 
  if (Operator == S_VODAFONE || Operator == S_VODAFONE_A || Operator == S_VODAFONE_C){

    // add AllThingsTalk credentials
    String deviceId = DEVICE_ID;    // defined in keys.h
    String token = DEVICE_TOKEN;
    TempBuf = deviceId + '\n' + token + '\n';

    for(cnt = 0 ; cnt < TempBuf.length(); cnt++)  SendBuf[cnt] = TempBuf[cnt];
    
    if (Operator == S_VODAFONE ) JASON_DataPayload();
    else if (Operator == S_VODAFONE_A) ABCL_DataPayload();
    else CBOR_DataPayload();
  }

  else if (Operator == S_TMOBILE){
    cnt = 0;
    JASON_DataPayload();
  }
  
  else if (Operator == S_TMOBILE_A){
    cnt = 0;
    ABCL_DataPayload();
  }

  else if (Operator == S_TMOBILE_C){
    cnt = 0;
    CBOR_DataPayload();
  }

  else if (Operator == S_KPN || Operator == S_KPN_A || Operator == S_KPN_C) {
    DEBUG_STREAM.println("KPN HAS NOT BEEN IMPLEMENTED YET");
    setLight(RED);    // indicate error
    return;
  }

  //*************** send to provider ******************
  int lengthSent = r4x.socketSend(socketID, API_ENDPOINT, API_PORT, SendBuf, cnt);
  r4x.socketClose(socketID);
  
  // only reset if succesfull
  if (cnt == lengthSent) {
    TotalMassPM1 = 0; TotalMassPM2 = 0; TotalMassPM10 = 0; SampleCnt = 0;
    startMillis = millis(); 
    setLight(OFF);
  }
  else {
    if (SKETCH_DEBUG) DEBUG_STREAM.println("Error during sending");
    setLight(RED);    // indicate error
  }
  
  if (SKETCH_DEBUG)  {
    
    DEBUG_STREAM.print("data sent: (HEX)  0x"); 
    for (byte i= 0; i < cnt; i++) {
      if (SendBuf[i] < 0x10) DEBUG_STREAM.print("0");
      DEBUG_STREAM.print(SendBuf[i], HEX); 
      DEBUG_STREAM.print(" ");
    }
    
    DEBUG_STREAM.print("\ndata sent: (ASCII)   "); 
    for (byte i= 0; i < cnt; i++) {
      if (SendBuf[i] > 0x1f && SendBuf[i] < 0x7f){
        DEBUG_STREAM.print( (char) SendBuf[i]);
        DEBUG_STREAM.print("  ");
      }
      else DEBUG_STREAM.print("?  ");
    }
         
    DEBUG_STREAM.print("\nLength buffer vs sent:"); DEBUG_STREAM.print(cnt); DEBUG_STREAM.print(",");
    DEBUG_STREAM.println(lengthSent); DEBUG_STREAM.println();
  }
}

//***********************************************************
//*            add CBOR data format                        **
//***********************************************************
void CBOR_DataPayload()
{
#ifdef INCLUDE_CBOR  
  // Data is a dictionary of key/values, which corresponds to CBORPair objects
  // Thus, we create a CBORPair, and reserve a buffer of 50 bytes for it.
  // This is not mandatory, as CBORPair can reallocate a bigger buffer on the
  // fly, but preferable in terms of memory management.
  CBORPair data = CBORPair(50);

  data.append("ID", SPS_id);
  data.append("M1", (float) TotalMassPM1 / SampleCnt);
  data.append("M2", (float) TotalMassPM2 / SampleCnt);
  data.append("M10",(float) TotalMassPM10 / SampleCnt);

  // get start of buffer
  const uint8_t *cbor_encoded = data.to_CBOR();

  // copy result to sent buffer
  for (size_t i=0 ; i < data.length() ; i++) {
      SendBuf[cnt++] = cbor_encoded[i];
  }
#endif
}

//***********************************************************
//*            add JASON data format                       **
//***********************************************************
void JASON_DataPayload(){
  
  TempBuf =  "{\"ID\":{\"value\":\"" + String(SPS_id) + "\"}";
  TempBuf += ",\"M1\":{\"value\":" + String(TotalMassPM1 / SampleCnt) +"}";
  TempBuf += ",\"M2\":{\"value\":" + String(TotalMassPM2 / SampleCnt) +"}";
  TempBuf += ",\"M10\":{\"value\":" + String(TotalMassPM10 / SampleCnt) +"}}";
  
  // copy result to sent buffer
  for(uint8_t i = 0 ; i < TempBuf.length(); i++)  SendBuf[cnt++] = TempBuf[i];
}

//*********************************************************************
//* add AllThingsTalk Binary Conversion Language​(ABCL)​data format  **
//*********************************************************************
void ABCL_DataPayload()
{
  // add ID
  for (uint8_t i = 0; i < SPSID; i++)  SendBuf[cnt++] = SPS_id[i];
 
  // add value
  add_float(TotalMassPM1 / SampleCnt);
  add_float(TotalMassPM2 / SampleCnt);
  add_float(TotalMassPM10 / SampleCnt);
}

//***********************************************************
//*  add floating as 4 bytes to the send buffer            **
//***********************************************************
void add_float(float c)
{
  ByteToFloat conv;
  conv.value = c;
  for (byte i = 0; i < 4; i++)
    SendBuf[cnt++] = conv.array[3-i];
}

//*************************************************************
//**                  SETUP FUNCTIONS                        **
//*************************************************************
void setup() {
  
  sodaq_wdt_safe_delay(STARTUP_DELAY);
  
  InitLed(); 
   
  DEBUG_STREAM.begin(DEBUG_STREAM_BAUD);
 
  if (SKETCH_DEBUG || SPS30_DEBUG || SODAQ_DEBUG) 
    serialTrigger((char *) "SPS30-SODAQ-EXAMPLE1 : Basic values. press <enter> to start");

#ifndef INCLUDE_CBOR  
  if (Operator == S_VODAFONE_C || Operator == S_TMOBILE_C || Operator == S_KPN_C)
  Errorloop((char *)"REQUESTED CBOR HAS NOT BEEN SELECTED TO BE INCLUDED",0);
#endif
   
  if (Operator == S_VODAFONE || Operator == S_TMOBILE || Operator == S_KPN)
    DEBUG_STREAM.println(F("Sending in Jason data format"));

  else if (Operator == S_VODAFONE_A || Operator == S_TMOBILE_A || Operator == S_KPN_A)
    DEBUG_STREAM.println(F("Sending in ABCL data format"));

  else if (Operator == S_VODAFONE_C || Operator == S_TMOBILE_C || Operator == S_KPN_C)
    DEBUG_STREAM.println(F("Sending in CBOR data format"));

  else
    Errorloop((char *) "Unknown Data format", 0);

  setLight(YELLOW);     InitSpS30();
  
  setLight(BLUE);       InitSodaq();

  setLight(OFF);

  // start measurement
  if (! sps30.start() ) Errorloop((char *) "Could NOT start measurement", 0);
    
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
  // startup time is less than 8 seconds, 10 has been used to be save
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
    // To prevent unnecessary stopping of fan.
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

//*********************************************************
//**        read and display all SPS30 values            **     
//*********************************************************
bool read_all() {
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {
      ret = sps30.GetValues(&val);
  
      // data might not have been ready
      if (ret == ERR_DATALENGTH || val.MassPM1 == 0) {
  
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
    DEBUG_STREAM.println(F("----------------------------Mass -----------------------------    -------------------------------- Number --------------------------------------       --Partsize --"));
    DEBUG_STREAM.println(F("                     Concentration [μg/m3]                                                 Concentration [#/cm3]                                           [μm]  "));
    DEBUG_STREAM.println(F(" PM1.0             PM2.5           PM4.0           PM10             PM0.5           PM1.0           PM2.5           PM4.0              PM10               Typical"));
    header = false;
  }

  print_aligned((double) val.MassPM1, 8, 5);   print_aligned((double) val.MassPM2, 8, 5);  print_aligned((double) val.MassPM4, 8, 5);
  print_aligned((double) val.MassPM10, 8, 5);  print_aligned((double) val.NumPM0, 9, 5);   print_aligned((double) val.NumPM1, 9, 5);
  print_aligned((double) val.NumPM2, 9, 5);    print_aligned((double) val.NumPM4, 9, 5);   print_aligned((double) val.NumPM10, 15, 5);
  print_aligned((double) val.PartSize, 7, 5);  DEBUG_STREAM.print(F("\n"));

  // collect data for sending later on
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
//**                   INITIALIZE SODAQ                         **
//****************************************************************
void InitSodaq() {
  DEBUG_STREAM.print("Initializing and connecting .. ");
  DEBUG_STREAM.println(provider);

  if (Operator == S_VODAFONE || Operator == S_VODAFONE_A || Operator == S_VODAFONE_C){
    if(! strcmp(DEVICE_ID,"yourdeviceid"))
      Errorloop((char *) "Please update Vodafone credentials in keys.h", 0);
  }
  
  MODEM_STREAM.begin(r4x.getDefaultBaudrate());   // set to 115200 in r4x.h
  
  if (SODAQ_DEBUG) r4x.setDiag(DEBUG_STREAM);
  r4x.init(&saraR4xxOnOff, MODEM_STREAM);
  
  if (!r4x.connect(apn, urat, SIM_ICCID,forceOperator, BAND_MASK_UNCHANGED, BAND_MASK_UNCHANGED)) 
    Errorloop((char *) "FAILED TO CONNECT TO MODEM", 0);
}

//***********************************************************
//*            read and display SPS30 device info          **
//***********************************************************
void GetDeviceInfo() {

  char buf[32];
  uint8_t ret, i, j=0;
  SPS30_version v;

  //try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == ERR_OK) {
    DEBUG_STREAM.print(F("Serial number : "));
    if(strlen(buf) > 0) {
      DEBUG_STREAM.print(buf);
      
      // use the last digits (4 = default) for SPS_id
      if (SPSID > strlen(buf)) ErrtoMess((char *) "Too many SPSID digits requested", 0);
      
      for (i = strlen(buf) - SPSID, j = 0; i < strlen(buf); i++, j++) SPS_id[j]= buf[i];
    }
    else DEBUG_STREAM.print(F("not available"));
  }
  else
    ErrtoMess((char *) "could not get serial number", ret);

  SPS_id[j] = 0x0; // terminate
  
  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == ERR_OK){
    DEBUG_STREAM.print(F("\tProduct name  : "));

    if(strlen(buf) > 0)  DEBUG_STREAM.print(buf);
    else DEBUG_STREAM.print(F("not available"));
  }
  else
    ErrtoMess((char *) "could not get product name.", ret);

  // try to get version info
  ret = sps30.GetVersion(&v);
  if (ret != ERR_OK) {
    DEBUG_STREAM.println(F("Can not read version info"));
    return;
  }

  DEBUG_STREAM.print("\tFirmware level: ");
  DEBUG_STREAM.print(v.major);
  DEBUG_STREAM.print(".");
  DEBUG_STREAM.println(v.minor);

  if (SP30_COMMS != I2C_COMMS) {
    DEBUG_STREAM.print("Hardware level: ");
    DEBUG_STREAM.print(v.HW_version);

    DEBUG_STREAM.print("\t\t\tSHDLC protocol: ");
    DEBUG_STREAM.print(v.SHDLC_major);
    DEBUG_STREAM.print(".");
    DEBUG_STREAM.print(v.SHDLC_minor);
    DEBUG_STREAM.print("\t\t");
  }
    
  DEBUG_STREAM.print("Library level : ");
  DEBUG_STREAM.print(v.DRV_major);
  DEBUG_STREAM.print(".");
  DEBUG_STREAM.println(v.DRV_minor);
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
    setLight(RED);    delay(1000);
    setLight(OFF);    delay(1000);
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
