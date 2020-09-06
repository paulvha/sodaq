/*
  MIT License

  Copyright (c) 2018 GabrielNotman

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  This sketch will display the GPS location and commands can be entered
  to retrieve other information
 */
#include <strings.h>
#include <Wire.h>

// SET TO 1 TO DISPLAY TIME, DATE, LAT AND LONG
// SET TO 0 TO DISPLAY ALL RAW INFO SENT BY GPS
#define PARSERESPONDS 1

///////////////////////////////////////////////////////////////////////
//                                                                   //
//           NO CHANGES NEEDED BEYOND THIS POINT                     //
//                                                                   //
///////////////////////////////////////////////////////////////////////

#define GPS_ADR 0x42                // I2C address
#define CONSOLE_STREAM SerialUSB    // console
#define CONSOLE_BAUD 115200
#define READ_TIMEOUT_MS 100         // number of milliseconds to wait for input console
#define BUFFER_SIZE 256             // buffer to hold input
uint8_t buffer[BUFFER_SIZE];

// store the GPS info
struct {
  char  UTC_time[7];      // zero terminate
  char  valid;
  float latitude;
  char  dir_lat;
  float longtitude;
  char  dir_long;
  char  UTC_date[7];      // zero terminate
} GPS;

void setup()
{
  CONSOLE_STREAM.begin(CONSOLE_BAUD);
  while (!CONSOLE_STREAM); //Wait for user to open terminal
    
  CONSOLE_STREAM.println("GPS info");

  Wire.begin();
  memset(buffer, 0, BUFFER_SIZE);

  serialTrigger((char *) "press <enter> to start");
  gpsEnable(true);      // set pin
}

void loop() 
{
  //Console -> GPS
  memset(buffer, 0, BUFFER_SIZE);
  writeGPS(readConsole());

  //GPS -> Console
  memset(buffer, 0, BUFFER_SIZE);
  writeConsole(readGPS());

  delay(1000);
}

/**
 * serialTrigger prints repeated message, then waits for enter
 * to come in from the serial port.
 */
void serialTrigger(char * mess)
{
  CONSOLE_STREAM.println();

  while (!CONSOLE_STREAM.available()) {
    CONSOLE_STREAM.println(mess);
    delay(2000);
  }

  while (CONSOLE_STREAM.available())
    CONSOLE_STREAM.read();
}

void gpsEnable(bool state)
{
  //Enable GPS module
  pinMode(GPS_ENABLE, OUTPUT);
  digitalWrite(GPS_ENABLE, state);
  delay(2000);
}

// read from GPS
size_t readGPS()
{
  return readUbloxI2cStream();
}

// read from console
size_t readConsole() 
{
  return readSerialStream((Stream*)&CONSOLE_STREAM);
}

// read from GPS
size_t readUbloxI2cStream()
{
  uint16_t count = 0;
  Wire.beginTransmission(GPS_ADR);
  Wire.write((uint8_t)0xFD);        // read available number of bytes
  Wire.endTransmission(false);
  Wire.requestFrom(GPS_ADR, 2);
  count = (uint16_t)(Wire.read() << 8) | Wire.read();
  count = (count > BUFFER_SIZE) ? BUFFER_SIZE : count;

  if (count) {
    for (size_t i = 0; i < (count-1); i++) {
      Wire.requestFrom(GPS_ADR, 1, false);
      buffer[i] = Wire.read();
    }
    Wire.requestFrom(GPS_ADR, 1);
    buffer[count-1] = Wire.read();
  }

  return count;
}

// read from console

size_t readSerialStream(Stream* stream) 
{
  uint32_t last = millis();
  size_t count = 0;
  while ((count < BUFFER_SIZE) && (millis() < (last + READ_TIMEOUT_MS))) {
    if (stream->available()) {
      buffer[count++] = stream->read();
      last = millis();
    }
  }
  return count;  
}

// write to GPS
void writeGPS(size_t count)
{
  Wire.beginTransmission(GPS_ADR);
  Wire.write(buffer, count);
  Wire.endTransmission();
}

// write to console RAW
bool writeConsole_org(size_t count)
{
  char linebuf[100];
  uint8_t j;
  
  for (size_t i = 0, j = 0; i < count; i++) {
    
    linebuf[j] = buffer[i];       // fill line buffer
    
    if (linebuf[j] == 0x0a) {     // end of line

      linebuf[j] = 0x0;           // terminate

      CONSOLE_STREAM.println(linebuf);
      j = 0;                     // restart line buffer
    }
    else
      j++;                       // next position line buffer
  }

  return false;
}


// write to console
bool writeConsole(size_t count)
{
  
  if (! PARSERESPONDS) return(writeConsole_org(count));
  
  char linebuf[100];
  uint8_t j;
  
  for (size_t i = 0, j = 0; i < count; i++) {
    
    linebuf[j] = buffer[i];       // fill line buffer
    
    if (linebuf[j] == 0x0a) {     // end of line

      linebuf[j] = 0x0;           // terminate
      
      if (strstr(linebuf, "$GNRMC")) {  // line with RMC data to parse ?
        return( ParsePositionInfo(linebuf));     // parse line
      }
      else
       j = 0;                     // restart line buffer
    }
    else
      j++;                       // next position line buffer
  }

  return false;
}

/* 
 *  @brief : parse the RMC – Recommended Minimum Specific GNSS Data
 *  
 * The input data is in the format of:
 * 
 * cnt:1     2      3  4         5   6         7  8    9   10   11  12 13
 * $GNRMC,204116.00,A,5218.56163,N,00439.53669,E,0.052, ,060420,  ,  , D*64
 * 
 * the output data is stored in the global GPS structure
 * 
 * return
 * True : data is valid
 * false : data is NOT valid
 */
bool ParsePositionInfo(char *p)
{
  uint8_t c = 1, i;
  char *token;

  GPS.UTC_time[0] = 0x0;
  GPS.UTC_date[0] = 0x0;
 
  while ((token = strsep(&p, ",")) != NULL) {
    
    switch(c){
        
      case 2:  // time : hhmmss.ss
        for(i = 0 ; i < 6 ; i++) GPS.UTC_time[i] = token[i];
        GPS.UTC_time[i] = 0x0;

        break;
      case 3:  // validate A = data is suitable, V = invalid
        GPS.valid = *token;
        break;

      case 4:  // latitude ddmm.mmmmm
        GPS.latitude = strtof(token, 0) / 100;
        break;

      case 5:  // direction N = north, S = south
        GPS.dir_lat = *token;
        break;

      case 6: // longtitude dddmm.mmmmm
        GPS.longtitude = strtof(token, 0) / 100;
        break;

      case 7: // direction E = east, W = west
        GPS.dir_long = *token;
        break;

      case 10 : //UTC date of position fix ddmmyy
        for(i = 0 ; i < 6 ; i++) GPS.UTC_date[i] = token[i];
        GPS.UTC_date[i] = 0x0;
        break;  

      //case 1:   // skip $GNRMC
      //case 8:   // skip speed over ground (knots)
      //case 9:   // skip course over ground
      //case 11:  // skip magnetic variation (degrees)
      //case 12:  // skip variation sense
      //case 13:  // skip differential mode + checksum
      default:
        break;
    }
    
    c++;          // count next entry
  }
  
  if (GPS.valid != 'A') {
    CONSOLE_STREAM.println("No valid positioning info yet");
    if (GPS.UTC_time[0] != 0x0) {
      CONSOLE_STREAM.print("UTC time\t");
      CONSOLE_STREAM.println(GPS.UTC_time);
    }
    if (GPS.UTC_date[0] != 0x0) {
      CONSOLE_STREAM.print("UTC date\t");
      CONSOLE_STREAM.println(GPS.UTC_date);
    }
  }
  else {
    CONSOLE_STREAM.print("Valid positioning\nUTC time\t");
    CONSOLE_STREAM.println(GPS.UTC_time);
    CONSOLE_STREAM.print("UTC date\t");
    CONSOLE_STREAM.println(GPS.UTC_date);
    CONSOLE_STREAM.print("Latitude\t");
    CONSOLE_STREAM.print(GPS.latitude,5);
    CONSOLE_STREAM.print(" ");
    CONSOLE_STREAM.println(GPS.dir_lat);
    CONSOLE_STREAM.print("Longtitude\t");      
    CONSOLE_STREAM.print(GPS.longtitude,5);
    CONSOLE_STREAM.print(" ");
    CONSOLE_STREAM.println(GPS.dir_long);    
  }
  CONSOLE_STREAM.println();
  
  if (GPS.valid == 'A') return true;
  return false;
}
