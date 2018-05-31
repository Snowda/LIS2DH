LIS2DH
======================

Arduino device driver for communicating with LIS2DH accelerometer over I2C

![LIS2DH datasheet (PDF)](http://www.st.com/web/en/resource/technical/document/datasheet/DM00042751.pdf)

======================
Status
======================

Library was uncomplete and no working. Not updated since 2014.
I'm updating it to make it a fully working library for LISD2D.
Actually the library is supporting most of the register configuration in raw mode and user friendly mode.
Have fun with It ! 
www.disk91.com

======================
Installation
======================

Requires the Arduino Libraries
![Wire.h](http://arduino.cc/en/reference/wire) 
for I2C and 
![SPI.h](http://arduino.cc/en/Reference/SPI) 
depending on how you wish to interface with the chip.

*warning* SPI not supported yet!

Download the repo as a zip file and install throught the Arduino IDE and select:

Sketch -> Import Library -> Add Library

Make sure that if you are installing updates that you remove any pre-existing libraries called "LIS2DH".

======================
Wiring Diagram
======================

![LIS2DH wiring diagram](docs/LIS2DH.png)

I2C lines: SDA and SCL