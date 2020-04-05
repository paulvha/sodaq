#ifndef KEYS_h
#define KEYS_h

// Enter your Allthingstalk device authentication
//char* DEVICE_ID = "yourdeviceid";
//char* DEVICE_TOKEN = "yourdevicetoken";

char* DEVICE_ID = "ZgOwGcS8J7MbR8xKxWTrdeqN";
char* DEVICE_TOKEN = "vodafone:4Vkw2VwzmGR9G0lqFyS9RSZLZkeenOY2uBARdhs";

// Enter the API endpoint for your Allthingstalk IP
char* ALLTHINGSTALK_IP = "37.97.216.22";

//*****************************************************************
//**                      LED FUNCTIONS                          **
//*****************************************************************
// define leD colors
enum lightColor {
  RED,
  GREEN,
  BLUE,
  YELLOW,
  MAGENTA,
  CYAN,
  WHITE,
  OFF
};

#if defined(_VARIANT_SODAQ_SFF_) || defined(_VARIANT_SODAQ_SARA_)
  #define ledRed    LED_RED
  #define ledGreen  LED_GREEN
  #define ledBlue   LED_BLUE
#else
  #define ledRed    RED
  #define ledGreen  GREEN
  #define ledBlue   BLUE
#endif // board variant

#endif
