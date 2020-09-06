/**
 * SODAQ Library Header file
 *
 * Copyright (c) September 2020, Paul van Haastrecht
 *
 * All rights reserved.
 *
 * Development environment specifics:
 * Arduino IDE 1.8.12 / 1.8.13
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/
#ifndef PVH_SODAQ_H
#define PVH_SODAQ_H

#include <Sodaq_R4X.h>
#include <Sodaq_wdt.h>

/**
 * library version levels
 */
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 0


//*****************************************************************
//**                      Operator info                          **
//*****************************************************************
enum Select_Operator {
  S_VODAFONE,        // sent in Jason format
  S_KPN,             // sent in Jason format
  S_TMOBILE,         // sent in Jason format
  S_VODAFONE_A,      // sent in AllThingsTalk Binary Conversion Language​(ABCL)​
  S_KPN_A,           // sent in AllThingsTalk Binary Conversion Language​(ABCL)​
  S_TMOBILE_A,       // sent in AllThingsTalk Binary Conversion Language​(ABCL)​
  S_VODAFONE_C,      // sent in Concise Binary Object Representation (CBOR)
  S_KPN_C,           // sent in Concise Binary Object Representation (CBOR)
  S_TMOBILE_C,       // sent in Concise Binary Object Representation (CBOR)
  S_NONE
};

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

void InitLed();
void setLight(lightColor color) ;

#endif /* PVH_SODAQ_H */
