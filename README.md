# msfs2020_gps_rx
This project has been created to facilitate a GPS messages receiver for MS Flightsimulator 2020 (MSFS2020) using a Raspberry Pi Pico microcontroller board (RPi Pico).
It is uses CircuitPython (CP) firmware, flashed into the RPi Pico.
It also uses two CP libraries: adafruit_bus_device (i2c_device.py) and lcd (i2c_pcf8574_interface.py / lcd.py).

Development has been done while running MSFS2020 on a desktop PC with MS Windows 10 Pro Operating System installed; a serial connection between the PC and a RPi Pico
through a USB-to-RS232 converter. At the same time running FSUIPC7, an add-on to MSFS2020. FSUIPC7 has a funcion called 'GPSout' which makes it possible to broadcast
GPS flight data using one or more selectable types of NMEA GPS message sentences.

The current edition of this GPS receiver is able to decode GPS NMEA message sentences of the types $GPGRC and $GPGGA. 
Both message types will be received in one reception iteration.
A UART of the RPi Pico is programmed to store the incoming data into a rx_buffer. After reception the data is analyzed and selectively saved to be displayed on an 4x20 LCD.
A Hitachi 44780 type 4x20 LCD is connected to the RPi Pico via an I2C wire connection.
The following GPS data will be displayed: Latitude, Longitude, Groundspeed, Course and Altitude.

In the past years I have created various models of these receivers for various types of platforms like: Raspberry Pi, Arduino MEGA2560 and Raspberry Pi Compute Model.
Initially I created these receivers for other flightsimulators: Prepar3D and X-Plane 11.

This is the first time that I create a GPS receiver for the new MSFS2020.
This version of the GPS message receiver has ben tested for many hours at various positions around the virtual globe to check if the presentation on the LCD would be correct
in various planet positions.

This is just a small project, my first published on GitHub. I published this repository here to not keep it to myself.
Feedback is always welcome.
Paulus Schulinck
(@paulsk on Discord / CircuitPython, also member of 'deepdivers')
