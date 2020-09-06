# NB-IoT examples with SODAQ

## ===========================================================

## Getting Started
After studying and working with Lora(wan) on different projects and having
working with a number of sensors, I decided to spend time on NB-IoT. This
is not a replacement of Lora, but each has it's unique place and you can
now choose the best solution.

### A word about uBlox experience
After a lot reading it seems that UBLOX has the good modules for this and
I choose the SARA R410M. The good news, this module is used on a wide
variety of boards. The bad news, if you get a module with old firmware,
the support (despite many mails) from UBLOX is bad, really very bad, read:
**the worst of all companies I have been dealing with**

They are an OEM supplier and do not want to support end users. New firmware
was available, but they did not want to provide it. They do NOT want to
help you. You had to go back to your supplier. Both the supplier and reseller
did not have it. I was stuck, I have wasted weeks and returned my board.

### A word about SODAQ
I connected with **SODAQ support department before buying and got extremely
good, honest and fast answers.** I decided to buy their SODAQ ARF/SARA board.
Great online forum, good open source software all you want. Got the first
project up and running in half a day.

### My Learning
Make sure to not only select the right technology, but as important choose
the right supplier.

## SODAQ Examples

I plan to create a number of different examples to test NB-IoT. The Sodaq
board I acquired comes with Vodafone backoffice ( Allthingstalk.com). I
am using that for now, as it working smooth and easy. In later
examples I want to look at others as well.

The projects :
### Example 1 : SPS30 sensor to SODAQ and Allthingstalk.
### Example 2 : SPS30 sensor + BME280 to SODAQ and Allthingstalk.
### Example 3 : Detect turning, shacking and tilting the board with Accelerator.
### Example 4 : SPS30 sensor + DS18x20 to SODAQ and Allthingstalk.
### Example 5 : SPS30 sensor + DHT-11 to SODAQ and Allthingstalk.
### Example 6 : SPS30 sensor + SCD30 to SODAQ and Allthingstalk.
### Example7_GPS
### ATT to Luftdaten: ATT forwarder to Luftdaten

<br> Many of the examples have a .odt (open document) file in the folder.
This contains more detailed information about the setup and project. They
can be read with nearly any word processor (LibreOffice, Microsoft word etc)

## Prerequisites
Documented in the top of each sketch and open document in folder

## Software installation
Obtain the zip and install like any other

## Program usage
### Program options
Please see the description in the top of the sketch and read the documentation (odt)

## Versioning

### version 1.0 / April 2020
 * Initial version with Example1, Example2 and Example3

### version 1.0.1 / April 2020
 * Update to example1, example2 (added SPS30 can be connected serial)
 * Added example4 and example7

### version 2.0 / June 2020
 * Added example5 + Documentation
 * Added MQTT forwarder AllThingsTalk to Luftdaten

### version 3.0 / September 2020
 * Added T-Mobile for Sodaq
 * Added ABCL and CBOR data format

## Author
 * Paul van Haastrecht (paulvha@hotmail.com)

## License
This project is licensed under the GNU GENERAL PUBLIC LICENSE 3.0

## Acknowledgments
Please make sure to read the Allthingstalk documentation, use the
information from Sodaq about the boards and examples ![https://learn.sodaq.com/Boards/](https://learn.sodaq.com/Boards/)
and use the Sodaq forum :![http://forum.sodaq.com/](http://forum.sodaq.com/)
