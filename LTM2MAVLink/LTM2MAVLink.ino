// Code from
// S.Port to MAVLink Converter
// https://github.com/davwys/arduino-sport-to-mavlink
// (c) 2019 David Wyss
//
// and
// 
// Ghettostation
// https://github.com/KipK/Ghettostation
// (c) by Guillaume S
//
// Base Arduino to MAVLink code from https://github.com/alaney/arduino-mavlink
// 
// Sample codes & inspirations from different projects:
// - multiwii (https://github.com/multiwii/multiwii-firmware)
// - minimosd-extra (http://code.google.com/p/minimosd-extra/)
// - UAVTalk implementation from minoposd (http://code.google.com/p/minoposd/)
// - NMEA & Ublox libraries from Jordi Muñoz and Jose Julio (DIYDrones.com)
// THANKS TO ANDREW TRIDGELL AND ALL THE ARDUPILOT TEAM FOR ALL GUIDANCE, SUPPORT AND INSPIRATIOON


//    Uncomment hardware target
#define ESP8266
//#define ProMini

#include <mavlink.h>

#ifdef ESP8266

    #include <ESP8266WiFi.h>
    #include <WiFiUdp.h>
    
    //    CONFIGURE Wifi
        
    unsigned int localUdpPort   = 14555;
    unsigned int remoteport     = 14550;
    const char *ssid            = "LTM2MAVLinkUDP";
    const char *password        = "password";         // set as required
    
    IPAddress AP_ip(192, 168, 4, 5);
    IPAddress AP_gateway(192, 168, 4, 1);
    IPAddress AP_subnet(255, 255, 255, 0);
    IPAddress UDP_broadcast(192, 168, 4, 255);
    
    HardwareSerial & inPort = Serial;
    WiFiUDP outPort;

#else

    #include <AltSoftSerial.h>
    
    AltSoftSerial inPort;  //   LTM input on RX pin 8, Tx on pin 9 not used
    HardwareSerial & outPort = Serial;

#endif


//    CONFIGURE BAUDRATES

#define baud_LTM_in      1200

#ifndef ESP8266

    #define baud_mavlink_out 57600
    
#endif

//    LTM INPUT FIXED TO PIN 8 (ALTSOFTSERIAL RX)
//    MAVLINK OUTPUT ON TX (HARDWARE SERIAL)
//    OUTPUT BAUDRATES > 38400 MIGHT CAUSE PERFORMANCE ISSUES

//    MAVLink system parameters, see common.h for further detail
uint8_t    system_id = 1;        // Leave at 1 unless you need a specific ID
uint8_t    component_id = 1;     // Leave at 1 unless specifically required
uint8_t    system_type = 1;      // 0 Generic, 1 Fixed wing, 2 Quadrotor, 3 Coax Helicopter, 4 Helicopter, 5 Ground installation, 6 GCS
uint8_t    autopilot_type = 3;   // Leave at 0 for generic autopilot with all capabilities, 3 = ArduPilot
uint8_t    base_mode = 1;        // 1 = custom mode enabled, 4 = auto mode, 8 = guided mode, 16 = stabilize mode, 64 = manual mode, 128 = safety armed
uint32_t   custom_mode = 0;      // see mavlink dialect message enums. uses ardupilot plane custom mode set     
uint8_t    system_state = 0;     // 0 = unknown, 3 = standby, 4 = active, 5 = critical
uint32_t   upTime = 0;           // Leave at 0 if not required otherwise

//    Flight parameters
int16_t    roll = 0;               // LTM [deg] -> MAVLink [rad]
int16_t    pitch = 0;              // LTM [deg] -> MAVLink [rad]
uint16_t   heading = 0;            // course over ground [degrees]
int32_t    lat = 0;
int32_t    lon = 0;
int32_t    alt = 0;                // LTM [m]   -> MAVLink [cm]
uint16_t   groundspeed = 0;        // LTM [m/s] -> MAVLink [cm/s]
uint8_t    airspeed = 0;           // LTM [m/s] -> MAVLink [cm/s]
uint8_t    gps_sats = 0;
uint8_t    fixType = 0;            // 0-1: no fix, 2: 2D fix, 3: 3D fix
uint16_t   voltage_battery = 0;    // [mV]
uint16_t   current_battery = 0;    // [mA]
uint8_t    rssi = 0;
uint8_t    armed = 0;
uint8_t    failsafe = 0;

unsigned long previousTime_1 = 0;
unsigned long previousTime_2 = 0;


//    Setup
void setup() {
    #ifdef ESP8266
    
        delay(500);
        inPort.begin(baud_LTM_in);
        WiFi.encryptionType(AUTH_WPA2_PSK);
        WiFi.softAPConfig(AP_ip, AP_gateway, AP_subnet);
        WiFi.softAP(ssid, password, 11);
        outPort.begin(localUdpPort);
    
    #else
    
        inPort.begin(baud_LTM_in);
        outPort.begin(baud_mavlink_out);
        
    #endif
}
 
// Main loop: read LTM and pack to MAVLink
void loop() {
    unsigned long currentTime = millis();
    
    if (currentTime - previousTime_1 >= 200) {
        
        // Read and convert LTM input
        ltm_check();
        ltm_read();

        // Send 5 Hz messages
        command_status(system_id, component_id, voltage_battery, current_battery);
        command_gps(system_id, component_id, upTime, fixType, lat, lon, alt, heading, groundspeed, gps_sats);
        command_attitude(system_id, component_id, upTime, roll, pitch, heading);
        command_vfr_hud(system_id, component_id, groundspeed, airspeed, heading, alt);
        command_rc_channels_raw(system_id, component_id, upTime, rssi);

        // reset 5 Hz timer
        previousTime_1 = currentTime;
    }

    if (currentTime - previousTime_2 >= 1000) {

        // Send heartbeat at 1 Hz
        command_heartbeat(system_id, component_id, system_type, autopilot_type, base_mode, custom_mode, system_state);

        // Reset 1 Hz timer
        previousTime_2 = currentTime;
    }
}

/* #################################################################################################################
 * LightTelemetry protocol (LTM)
 *
 * Ghettostation one way telemetry protocol for really low bitrates (1200/2400 bauds). 
 *         
 * Protocol details: 3 different frames, little endian.
 *   G Frame (GPS position) (2hz @ 1200 bauds , 5hz >= 2400 bauds): 18BYTES
 *    0x24 0x54 0x47 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF  0xFF   0xC0   
 *     $     T    G  --------LAT-------- -------LON---------  SPD --------ALT-------- SAT/FIX  CRC
 *   A Frame (Attitude) (5hz @ 1200bauds , 10hz >= 2400bauds): 10BYTES
 *     0x24 0x54 0x41 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xC0   
 *      $     T   A   --PITCH-- --ROLL--- -HEADING-  CRC
 *   S Frame (Sensors) (2hz @ 1200bauds, 5hz >= 2400bauds): 11BYTES
 *     0x24 0x54 0x53 0xFF 0xFF  0xFF 0xFF    0xFF    0xFF      0xFF       0xC0     
 *      $     T   S   VBAT(mv)  Current(ma)   RSSI  AIRSPEED  ARM/FS/FMOD   CRC
 * ################################################################################################################# */


#define LIGHTTELEMETRY_START1 0x24 //$
#define LIGHTTELEMETRY_START2 0x54 //T
#define LIGHTTELEMETRY_GFRAME 0x47 //G GPS + Baro altitude data ( Lat, Lon, Groundspeed, Alt, Sats, Sat fix)
#define LIGHTTELEMETRY_AFRAME 0x41 //A Attitude data ( Roll,Pitch, Heading )
#define LIGHTTELEMETRY_SFRAME 0x53 //S Sensors/Status data ( voltage, current, Rssi, Airspeed, Arm status, Failsafe status, Flight mode )
#define LIGHTTELEMETRY_GFRAMELENGTH 18
#define LIGHTTELEMETRY_AFRAMELENGTH 10
#define LIGHTTELEMETRY_SFRAMELENGTH 11

static uint8_t LTMserialBuffer[LIGHTTELEMETRY_GFRAMELENGTH-4];
static uint8_t LTMreceiverIndex;
static uint8_t LTMcmd;
static uint8_t LTMrcvChecksum;
static uint8_t LTMreadIndex;
static uint8_t LTMframelength;
static uint8_t crlf_count = 0;

uint8_t ltmread_u8() {
    return LTMserialBuffer[LTMreadIndex++];
}

uint16_t ltmread_u16() {
    uint16_t t = ltmread_u8();
    t |= (uint16_t)ltmread_u8()<<8;
    return t;
}

uint32_t ltmread_u32() {
    uint32_t t = ltmread_u16();
    t |= (uint32_t)ltmread_u16()<<16;
    return t;
}


void ltm_check() {
    LTMreadIndex = 0;

    if (LTMcmd==LIGHTTELEMETRY_GFRAME) {
        lat = ltmread_u32();
        lon = ltmread_u32();
        groundspeed = (uint16_t)ltmread_u8(); 
        alt = ltmread_u32() * 100;
        uint8_t ltm_satsfix = ltmread_u8();
        gps_sats = (ltm_satsfix >> 2) & 0xFF;
        fixType = ltm_satsfix & 0b00000011;
    }

    if (LTMcmd==LIGHTTELEMETRY_AFRAME) {
        pitch = ltmread_u16();
        roll = ltmread_u16();
        heading = ltmread_u16();
    }

    if (LTMcmd==LIGHTTELEMETRY_SFRAME) {
        //    static int frametick = 0;
        voltage_battery = ltmread_u16();
        current_battery = ltmread_u16();
        rssi = ltmread_u8();
        airspeed = ltmread_u8();
        uint8_t ltm_armfsmode = ltmread_u8();
        armed = (ltm_armfsmode & 0b00000001);
        failsafe = (ltm_armfsmode >> 1) & 0b00000001;
        custom_mode = (ltm_armfsmode >> 2) & 0b00111111;     
        if ((custom_mode == 0) || (custom_mode == 3) || (custom_mode == 4)) {
            base_mode = 65;
        } else if ((custom_mode == 2) || (custom_mode == 5) || (custom_mode == 6) || (custom_mode == 7) || (custom_mode == 8)) {
            base_mode = 17;
        } else if (10 <= custom_mode <= 15) {
            base_mode = 25;
        }
        if (armed == 1) {
            base_mode += 128;
            system_state = 4;
        } 
        if (failsafe == 1) {
            system_state = 5;
        }
    }
}

void ltm_read() {
    uint8_t c;

    static enum _serial_state {
        IDLE,
        HEADER_START1,
        HEADER_START2,
        HEADER_MSGTYPE,
        HEADER_DATA
    }

    c_state = IDLE;

    while (inPort.available()) {
        c = char(inPort.read());
    
        if (c_state == IDLE) {
            c_state = (c=='$') ? HEADER_START1 : IDLE;
        } else if (c_state == HEADER_START1) {
            c_state = (c=='T') ? HEADER_START2 : IDLE;
        } else if (c_state == HEADER_START2) {
            switch (c) {
                case 'G':
                LTMframelength = LIGHTTELEMETRY_GFRAMELENGTH;
                c_state = HEADER_MSGTYPE;
                break;
                   
                case 'A':
                LTMframelength = LIGHTTELEMETRY_AFRAMELENGTH;
                c_state = HEADER_MSGTYPE;
                break;
                    
                case 'S':
                LTMframelength = LIGHTTELEMETRY_SFRAMELENGTH;
                c_state = HEADER_MSGTYPE;
                break;
 
                default:
                c_state = IDLE;
            }

            LTMcmd = c;
            LTMreceiverIndex=0;
        } else if (c_state == HEADER_MSGTYPE) {
            if(LTMreceiverIndex == 0) {
                LTMrcvChecksum = c;
            } else {
                LTMrcvChecksum ^= c;
            }

            if(LTMreceiverIndex == LTMframelength-4) {   // received checksum byte
                if(LTMrcvChecksum == 0) {
                    ltm_check();
                    c_state = IDLE;
                } else {     // wrong checksum, drop packet
                    c_state = IDLE; 
                }
            } else LTMserialBuffer[LTMreceiverIndex++]=c;
        }
    }
}

/************************************************************
* @brief Sends a MAVLink heartbeat
* @param Basic UAV parameters, as defined above
* @return void
*************************************************************/

void command_heartbeat(uint8_t system_id, uint8_t component_id, uint8_t system_type, uint8_t autopilot_type, uint8_t base_mode, uint32_t custom_mode, uint8_t system_state) {

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
    // Pack the message
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, system_type, autopilot_type, base_mode, custom_mode, system_state);
 
    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
    // Send the message
    #ifdef ESP8266
    
        outPort.beginPacket(UDP_broadcast, remoteport);
        outPort.write(buf, len);
        outPort.endPacket();

    #else
    
        outPort.write(buf, len);

    #endif
}


/************************************************************
* @brief Send battery parameters
* @param 
* @return void
*************************************************************/
       
void command_status(uint8_t system_id, uint8_t component_id, uint16_t voltage_battery, uint16_t current_battery) {

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_sys_status_pack(system_id, component_id, &msg, 32767, 32767, 32767, 500, voltage_battery, current_battery, -1, 0, 0, 0, 0, 0, 0);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
    // Send the message (.write sends as bytes)
    #ifdef ESP8266
    
        outPort.beginPacket(UDP_broadcast, remoteport);
        outPort.write(buf, len);
        outPort.endPacket();

    #else
    
        outPort.write(buf, len);

    #endif
}


/************************************************************
* @brief Sends location (GPS position), altitude and heading
* @param lat: latitude in degrees, lon: longitude in degrees, alt: altitude, heading: heading
* @return void
*************************************************************/

void command_gps(uint8_t system_id, uint8_t component_id, uint32_t upTime,  uint8_t fixType, int32_t lat, int32_t lon, int32_t alt, uint16_t groundspeed, uint16_t heading, uint8_t gps_sats) {

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg, upTime, fixType, lat, lon, alt, 65535, 65535, groundspeed, heading * 100, gps_sats);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
    // Send the message (.write sends as bytes)
    #ifdef ESP8266
    
        outPort.beginPacket(UDP_broadcast, remoteport);
        outPort.write(buf, len);
        outPort.endPacket();

    #else
    
        outPort.write(buf, len);

    #endif
}


/************************************************************
* @brief Send attitude and heading data to the primary flight display (artificial horizon)
* @param roll, pitch, yaw
* @return void
*************************************************************/
       
void command_attitude(uint8_t system_id, uint8_t component_id, int32_t upTime, int16_t roll, int16_t pitch, int16_t heading) {

    //Radian -> degree conversion rate
    float radian = 57.2958;
    float rollrad = (float)roll / radian;
    float pitchrad = (float)pitch / radian;
    float yawrad = (float)heading / radian;

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_attitude_pack(system_id, component_id, &msg, upTime, rollrad, pitchrad, yawrad, 0, 0, 0); 

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes)
    #ifdef ESP8266
    
        outPort.beginPacket(UDP_broadcast, remoteport);
        outPort.write(buf, len);
        outPort.endPacket();

    #else
    
        outPort.write(buf, len);

    #endif
}


/************************************************************
* @brief Send vfr_hud
* @param groundspeed, airspeed, heading, altitude MSL
* @return void
*************************************************************/

void command_vfr_hud(uint8_t system_id, uint8_t component_id, float groundspeed, float airspeed, int16_t heading, float alt) {
  
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_vfr_hud_pack(system_id, component_id, &msg, groundspeed, airspeed, heading, 0, alt, 0); 

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes)
    #ifdef ESP8266
    
        outPort.beginPacket(UDP_broadcast, remoteport);
        outPort.write(buf, len);
        outPort.endPacket();

    #else
    
        outPort.write(buf, len);

    #endif
}


/************************************************************
* @brief Send rc_channels_raw
* @param rssi
* @return void
*************************************************************/

void command_rc_channels_raw(uint8_t system_id, uint8_t component_id, int32_t upTime, uint8_t rssi) {
  
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_rc_channels_raw_pack(system_id, component_id, &msg, upTime, 0, 0, 0, 0, 0, 0, 0, 0, 0, rssi); 

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes)
    #ifdef ESP8266
    
        outPort.beginPacket(UDP_broadcast, remoteport);
        outPort.write(buf, len);
        outPort.endPacket();

    #else
    
        outPort.write(buf, len);

    #endif
}
