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


#include <mavlink.h>
#include <AltSoftSerial.h>

//    CONFIGURE BAUDRATES
//    LTM INPUT FIXED TO PIN 8 (ALTSOFTSERIAL RX)
//    MAVLINK OUTPUT ON TX (HARDWARE SERIAL)

#define baud_mavlink_out 19200
#define baud_LTM_in      2400

// Parameter setup
// If the input protocol does not provide values for all of these, parameters will be left at their default values.
// All parameters set here are passed onward through serial output in MAVLink format.

//Basic UAV Parameters
uint8_t system_id = 1;        // MAVLink system ID. Leave at 0 unless you need a specific ID.
uint8_t component_id = 0;     // Should be left at 0. Set to 190 to simulate mission planner sending a command
uint8_t system_type = 1;      // UAV type. 0 = generic, 1 = fixed wing, 2 = quadcopter, 3 = helicopter
uint8_t autopilot_type = 0;   // Autopilot type. Usually set to 0 for generic autopilot with all capabilities
uint8_t system_mode = 64;     // Flight mode. 4 = auto mode, 8 = guided mode, 16 = stabilize mode, 64 = manual mode
uint32_t custom_mode = 0;     // Usually set to 0          
uint8_t system_state = 4;     // 0 = unknown, 3 = standby, 4 = active
uint32_t upTime = 0;          // System uptime, usually set to 0 for cases where it doesn't matter

//
// Flight parameters
int16_t roll = 0;         // Roll angle in degrees
int16_t pitch = 0;        // Pitch angle in degrees
int16_t heading = 0;    // Geographical heading angle in degrees
int32_t lat = 0;        // GPS latitude in degrees (example: 47.123456)
int32_t lon = 0;        // GPS longitude in degrees
int32_t alt = 0;        // Relative flight altitude in m
uint8_t groundspeed = 0; // Groundspeed in m/s
uint8_t airspeed = 0;    // Airspeed in m/s

// GPS parameters
uint8_t gps_sats = 0;     // Number of visible GPS satellites
uint8_t fixType = 0;      // GPS fix type. 0-1: no fix, 2: 2D fix, 3: 3D fix

// Battery parameters
uint16_t voltage_battery = 0;    // Battery voltage in V
uint16_t current_battery = 0;    // Battery current in A
uint16_t battery_consumed = 0;   // mAh used

uint8_t rssi = 0;
uint8_t armed = 0;
uint8_t failsafe = 0;
uint8_t mode = 0;

AltSoftSerial softSerial;

void setup() {
    Serial.begin(baud_mavlink_out);
    //Serial.println("This is hardware serial for mavlink output");
    softSerial.begin(baud_LTM_in);
    //softSerial.println("This is software serial for LTM input");
}
 
// Main loop: read LTM and pack to MAVLink
void loop() {
  
    // Read LTM input
    ltm_check();
    ltm_read();
    
    // Send MAVLink heartbeat
    command_heartbeat(system_id, component_id, custom_mode, system_type, autopilot_type, system_mode, system_state);
    
    //Send battery status
    command_status(system_id, component_id, voltage_battery, current_battery);
    
    // Send GPS and altitude data
    command_gps(system_id, component_id, upTime, fixType, lat, lon, alt, heading, groundspeed, gps_sats);
    
    
    // Send attitude data to artificial horizon
    command_attitude(system_id, component_id, upTime, roll, pitch);

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
#define LIGHTTELEMETRY_SFRAME 0x53 //S Sensors/Status data ( VBat, Consumed current, Rssi, Airspeed, Arm status, Failsafe status, Flight mode )
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
        groundspeed = ltmread_u8(); 
        alt = ltmread_u32();
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
        static int frametick = 0;
        voltage_battery = ltmread_u16();
        current_battery = ltmread_u16();
        rssi = ltmread_u8();
        airspeed = ltmread_u8();
        uint8_t ltm_armfsmode = ltmread_u8();
        armed = (bool)(ltm_armfsmode & 0b00000001);
        failsafe = (ltm_armfsmode >> 1) & 0b00000001;
        mode = (ltm_armfsmode >> 2) & 0b00111111;     
        frametick = millis();
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

    while (softSerial.available()) {
        c = char(softSerial.read());
    
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

void command_heartbeat(uint8_t system_id, uint8_t component_id, uint32_t custom_mode, uint8_t system_type, uint8_t autopilot_type, uint8_t system_mode, uint8_t system_state) {

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
    // Pack the message
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, custom_mode, system_type, autopilot_type, system_mode, system_state);
 
    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
    // Send the message
    Serial.write(buf, len);
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
    mavlink_msg_sys_status_pack(system_id, component_id, &msg, 32767, 32767, 32767, 500, voltage_battery, current_battery, 0, 0, 0, 0, 0, 0, -1);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
    // Send the message (.write sends as bytes)
    Serial.write(buf, len);
}


/************************************************************
* @brief Sends location (GPS position), altitude and heading
* @param lat: latitude in degrees, lon: longitude in degrees, alt: altitude, heading: heading
* @return void
*************************************************************/

void command_gps(int8_t system_id, int8_t component_id, uint32_t upTime, int32_t lat, int32_t lon, int32_t alt,  uint8_t groundspeed, int16_t heading, uint8_t fixType, uint8_t gps_sats) {

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg, upTime, fixType, lat, lon, alt, UINT16_MAX, UINT16_MAX, groundspeed, heading * 100, gps_sats);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
    // Send the message (.write sends as bytes)
    Serial.write(buf, len);
}


/************************************************************
* @brief Send attitude and heading data to the primary flight display (artificial horizon)
* @param roll, pitch, yaw
* @return void
*************************************************************/
       
void command_attitude(int8_t system_id, int8_t component_id, int32_t upTime, int16_t roll, int16_t pitch) {

    //Radian -> degree conversion rate
    float radian = 57.2958;
    float rollrad = (float)roll / radian;
    float pitchrad = (float)pitch / radian;

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_attitude_pack(system_id, component_id, &msg, upTime, rollrad, pitchrad, 0, 0, 0, 0); 

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes)
    Serial.write(buf, len);
}
