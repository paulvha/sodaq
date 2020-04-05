/*  demo the accelerator motion detection
 *  
 *  This sketch will do :
 *  
 *  It will initialise the accelerator.
 *  It will read and display the temperature
 *  it will read and display the accelerator movements
 *  if you put the SODAQ board on a flat place:the led will be  off
 *  if you tilt the front with the Grove connectors up: the led will turn BLUE
 *  if you hold the front with the Grove connectors down: the led will turn YELLOW
 *  if you turn over on it's side, the led will turn GREEN 
 *  if you shake the board left / right the led will turn WHITE
 *  
 *  Led functions is based on the LED.h / LED.CPP from the Rapid Development Kit Allthingstalk
 *  
 *  paulvha / Version 1.0 / April 2020 
 *  - Initial version
 */

#include <Sodaq_LSM303AGR.h>

// define lef colors
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

// define time (mS) delay after detection and setting led
// higher number give more time to settle the board in position
#define RESPONDSDELAY 800

// define the axes detection sensitivity
// higher number ==> less sensitive
#define XaxeSens 0.4    // tilt detection
#define YaxeSens 0.6    // shake detection
#define ZaxeSens 0.9    // turn detection

Sodaq_LSM303AGR accel;

//*****************************************************************
//**                      SETUP FUNCTIONS                        **
//*****************************************************************

void setup() {
  SerialUSB.begin(115200);
  delay(1000);
  
  SerialUSB.println("BEGIN");
  Wire.begin();
  
  if (accel.checkWhoAmI())  
    SerialUSB.println("FOUND ACCEL!");
  else {
    SerialUSB.println("NO ACCEL!. On Hold");
    while(1);
  }
  
  // init pins
  InitLed();
  
  accel.rebootAccelerometer();
  delay(1000);
  
  accel.enableAccelerometer(Sodaq_LSM303AGR::NormalMode, Sodaq_LSM303AGR::HrNormalLowPower100Hz, 
                            Sodaq_LSM303AGR::XYZ, Sodaq_LSM303AGR::Scale2g, true);
}

//*****************************************************************
//**                      lOOP FUNCTIONS                         **
//*****************************************************************

void loop() {
  double accZ, accX, accY;
  
  accX = accel.getX();
  accY = accel.getY();
  accZ = accel.getZ();
  
  SerialUSB.print("Temp: ");SerialUSB.print(accel.getTemperature());SerialUSB.print("C  ");
  SerialUSB.print("X: ");SerialUSB.print(accX); SerialUSB.print("  ");
  SerialUSB.print("Y: ");SerialUSB.print(accY); SerialUSB.print("  ");
  SerialUSB.print("Z: ");SerialUSB.print(accZ); SerialUSB.print("  ");SerialUSB.println("m/s^2");

  // tilt detection
  if (accX < -XaxeSens) setLight(BLUE);
  if (accX > XaxeSens ) setLight(YELLOW);

  // shake detection
  if (accY > YaxeSens || accY < -YaxeSens) {
    // only if flat on the surface
    if (accZ > ZaxeSens ) setLight(WHITE); 
  }

  // turned detection
  if (accZ < -ZaxeSens) setLight(GREEN);

  // upside position
  if (accZ > ZaxeSens ) setLight(OFF);

  // delay before next try
  delay(500);
}

//*****************************************************************
//**                      LED FUNCTIONS                          **
//*****************************************************************

#if defined(_VARIANT_SODAQ_SFF_) || defined(_VARIANT_SODAQ_SARA_)
  #define ledRed    LED_RED
  #define ledGreen  LED_GREEN
  #define ledBlue   LED_BLUE
#else
  #define ledRed    RED
  #define ledGreen  GREEN
  #define ledBlue   BLUE
#endif

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
  bool Rdelay = true;
  
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
      Rdelay = false;
      break;
  }
  
  // slow down responds
  if (Rdelay) delay(RESPONDSDELAY);
}
