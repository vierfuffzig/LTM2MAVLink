# LTM2MAVLink

A non-coder's approach at a LTM to MAVLink converter. 
Aimed at passing LTM downlink telemetry to MAVLink-compatible GCSes like MissionPlanner, QGroundCrontrol etc.

Tested on Arduino Pro Mini and ESP8266


Arduino Pro Mini:
-----------------
LTM telemetry input via AltSoftSerial on Pin D8, default 1200 baud

MAVLink output via hardware serial, default 57600 baud

ESP8266: 
--------
LTM telemetry input via HardwareSerial, default 1200 baud

MAVLink via UDP broadcast, SSID = LTM2MAVLinkUDP, password = password

adjust as required in config section


Code from:
----------

S.Port to MAVLink Converter: https://github.com/davwys/arduino-sport-to-mavlink (c) 2019 David Wyss

and

Ghettostation https://github.com/KipK/Ghettostation (c) by Guillaume S

Base Arduino to MAVLink code from https://github.com/alaney/arduino-mavlink

Sample codes & inspirations from different projects:
- multiwii (https://github.com/multiwii/multiwii-firmware)
- minimosd-extra (http://code.google.com/p/minimosd-extra/)
- UAVTalk implementation from minoposd (http://code.google.com/p/minoposd/)
- NMEA & Ublox libraries from Jordi Muñoz and Jose Julio (DIYDrones.com)

THANKS TO ANDREW TRIDGELL AND ALL THE ARDUPILOT TEAM FOR ALL GUIDANCE, SUPPORT AND INSPIRATION
