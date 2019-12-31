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

#define baud_mavlink_out 57600
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
float roll = 10;         // Roll angle in degrees
float pitch = 20;        // Pitch angle in degrees
float yaw = 0;          // Yaw angle in degrees
int16_t heading = 0;    // Geographical heading angle in degrees
float lat = 0.0;        // GPS latitude in degrees (example: 47.123456)
float lon = 0.0;        // GPS longitude in degrees
float alt = 0.0;        // Relative flight altitude in m
float lat_temp = 0.0;   //Temp values for GPS coords
float lon_temp = 0.0; 
float groundspeed = 0.0; // Groundspeed in m/s
float airspeed = 0.0;    // Airspeed in m/s
float climbrate = 0.0;   // Climb rate in m/s, currently not working
float throttle = 0.0;    // Throttle percentage

// GPS parameters
int16_t gps_sats = 0;     // Number of visible GPS satellites
int32_t gps_alt = 0.0;    // GPS altitude (Altitude above MSL)
float gps_hdop = 100.0;   // GPS HDOP
uint8_t fixType = 0;      // GPS fix type. 0-1: no fix, 2: 2D fix, 3: 3D fix

// Battery parameters
float battery_remaining = 0.0;  // Remaining battery percentage
float voltage_battery = 9.0;    // Battery voltage in V
float current_battery = 0.0;    // Battery current in A
float battery_consumed = 0.0;   // mAh used

uint8_t rssi = 0;
uint8_t armed = 0;
uint8_t failsafe = 0;
uint8_t mode = 0;

// SoftwareSerial on Pins D5 & D6
// SoftwareSerial BTSerial(5,6);


AltSoftSerial softSerial;

void setup() {
    Serial.begin(baud_mavlink_out);
    Serial.println("This is hardware serial for mavlink output");
    softSerial.begin(baud_LTM_in);
    softSerial.println("This is software serial for LTM input");
}
 
// Main loop: read LTM and pack to MAVLink
void loop() {
    // Read LTM input
    ltm_read();
    ltm_check();

    // Send MAVLink heartbeat
    command_heartbeat(system_id, component_id, system_type, autopilot_type, system_mode, custom_mode, system_state);
    
    //Send battery status
    command_status(system_id, component_id, battery_remaining, voltage_battery, current_battery);
    
    // Send GPS and altitude data
    command_gps(system_id, component_id, upTime, fixType, lat, lon, alt, gps_alt, heading, groundspeed, gps_hdop, gps_sats);
    
    // Send HUD data (speed, heading, climbrate etc.)
    command_hud(system_id, component_id, airspeed, groundspeed, heading, throttle, alt, climbrate);
    
    // Send attitude data to artificial horizon
    command_attitude(system_id, component_id, upTime, roll, pitch, yaw);

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
#define LIGHTTELEMETRY_GFRAME 0x47 //G GPS + Baro altitude data ( Lat, Lon, Speed, Alt, Sats, Sat fix)
#define LIGHTTELEMETRY_AFRAME 0x41 //A Attitude data ( Roll,Pitch, Heading )
#define LIGHTTELEMETRY_SFRAME 0x53 //S Sensors/Status data ( VBat, Consumed current, Rssi, Airspeed, Arm status, Failsafe status, Flight mode )
// #define LIGHTTELEMETRY_OFRAME 0x4F  //O OSD additionals data ( home pos, home alt, ddirection to home )
#define LIGHTTELEMETRY_GFRAMELENGTH 18
#define LIGHTTELEMETRY_AFRAMELENGTH 10
#define LIGHTTELEMETRY_SFRAMELENGTH 11
// #define LIGHTTELEMETRY_OFRAMELENGTH 18

static uint8_t LTMserialBuffer[LIGHTTELEMETRY_GFRAMELENGTH-4];
static uint8_t LTMreceiverIndex;
static uint8_t LTMcmd;
static uint8_t LTMrcvChecksum;
static uint8_t LTMreadIndex;
static uint8_t LTMframelength;
static uint8_t LTMpassed= 0 ;
static uint8_t LTM_ok = 0;
static uint8_t crlf_count = 0;
static uint8_t lastLTMpacket = 0;
void uploadFont();

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


//    uint32_t currentTime, displayTime;
//    // Display data once a second to not interfeere with data decoding
//    currentTime = millis();
//    if(currentTime > displayTime) {
//    displayTime = currentTime + 1000;
//    //  insert function here
//    }


void ltm_check() {
    LTMreadIndex = 0;
    LTM_ok = 1;
    lastLTMpacket = millis();

    if (LTMcmd==LIGHTTELEMETRY_GFRAME) {
        lat = (int32_t)ltmread_u32() / 10000000.0;
        lon = (int32_t)ltmread_u32() / 10000000.0;
        groundspeed = (float)(ltmread_u8()); 
        alt = ((int32_t)ltmread_u32()) / 100.0f;      // altitude from cm to m.
        uint8_t ltm_satsfix = ltmread_u8();
        gps_sats = (ltm_satsfix >> 2) & 0xFF;
        fixType = ltm_satsfix & 0b00000011;
        //memset(LTMserialBuffer, 0, LIGHTTELEMETRY_GFRAMELENGTH-4); 
        LTMpassed = 1;
    }

    if (LTMcmd==LIGHTTELEMETRY_AFRAME) {
        pitch = (int16_t)ltmread_u16();
        roll = (int16_t)ltmread_u16();
        heading = (float)(int16_t)ltmread_u16();
        if (heading < 0 ) heading = heading + 360.0f; //convert from -180/180 to 0/360°
        LTMpassed = 1;
    }

    if (LTMcmd==LIGHTTELEMETRY_SFRAME) {
        static int frametick = 0;
        // static uint16_t osd_curr_A_prev= 0;
        voltage_battery = (float) ltmread_u16() / 1000.0f;
        battery_consumed = ltmread_u16(); 
        // current_battery   = battery_used *360000 / (millis() - frametick);
        // battery_remaining = BAT_CAPACITY - mah_used;    
        rssi = ltmread_u8();
        airspeed = ltmread_u8();
        uint8_t ltm_armfsmode = ltmread_u8();
        armed = (bool)(ltm_armfsmode & 0b00000001);
        failsafe = (ltm_armfsmode >> 1) & 0b00000001;
        mode = (ltm_armfsmode >> 2) & 0b00111111;     
        frametick = millis();
        LTMpassed = 1;
    }

    //    if (LTMcmd==LIGHTTELEMETRY_OFRAME) {
    //        osd_home_lat = (int32_t)ltmread_u32() / 10000000.0;
    //        osd_home_lon = (int32_t)ltmread_u32() / 10000000.0;
    //        osd_home_alt = (int32_t)(ltmread_u32()) / 100.0f; // altitude from cm to m.
    //        osd_enabled  = ltmread_u8();
    //        osd_got_home = ltmread_u8();
    //        if (osd_enabled == 0) osd_clear = 1;
    //        LTMpassed = 1;
    //    }
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
                    
                //  case 'O':
                //  LTMframelength = LIGHTTELEMETRY_OFRAMELENGTH;
                //  c_state = HEADER_MSGTYPE;
                //  break;
 
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

void command_heartbeat(uint8_t system_id, uint8_t component_id, uint8_t system_type, uint8_t autopilot_type, uint8_t system_mode, uint32_t custom_mode, uint8_t system_state) {

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
    // Pack the message
    mavlink_msg_heartbeat_pack(system_id,component_id, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
 
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
       
void command_status(uint8_t system_id, uint8_t component_id, float battery_remaining, float voltage_battery, float current_battery) {

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_sys_status_pack(system_id, component_id, &msg, 32767, 32767, 32767, 500, voltage_battery * 1000.0, current_battery * 100.0, battery_remaining, 0, 0, 0, 0, 0, 0);

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

void command_gps(int8_t system_id, int8_t component_id, int32_t upTime, int8_t fixType, float lat, float lon, float alt, float gps_alt, int16_t heading, float groundspeed, float gps_hdop, int16_t gps_sats) {

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg, upTime, fixType, lat * 10000000.0, lon * 10000000.0, alt * 1000.0, gps_hdop * 100.0, 65535, groundspeed, 65535, gps_sats);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    //Send globalgps command
    command_globalgps(system_id, component_id, upTime, lat, lon, alt, gps_alt, heading);
  
    // Send the message (.write sends as bytes)
    Serial.write(buf, len);
}

/************************************************************
* @brief Send VFR_HUD core data
* @param 
* @return void
*************************************************************/
       
void command_hud(int8_t system_id, int8_t component_id, float airspeed, float groundspeed, int16_t heading, float throttle, float alt, float climbrate) {

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_vfr_hud_pack(system_id, component_id, &msg, airspeed, groundspeed, heading, throttle, alt * 1000.0, climbrate);

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
       
void command_attitude(int8_t system_id, int8_t component_id, int32_t upTime, float roll, float pitch, float yaw) {

    //Radian -> degree conversion rate
    float radian = 57.2958;

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_attitude_pack(system_id, component_id, &msg, upTime, roll/radian, pitch/radian, yaw/radian, 0, 0, 0); 

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes)
    Serial.write(buf, len);
}



/************************************************************
* @brief Sends Integer representation of location
* @param lat: latitude, lon: longitude, alt: altitude, gps_alt: altitude above MSL, heading: heading
* @return void
*************************************************************/
       
void command_globalgps(int8_t system_id, int8_t component_id, int32_t upTime, float lat, float lon, float alt, float gps_alt, uint16_t heading) {

    int16_t velx = 0; //x speed
    int16_t vely = 0; //y speed
    int16_t velz = 0; //z speed


    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_global_position_int_pack(system_id, component_id, &msg, upTime, lat * 10000000.0, lon * 10000000.0, gps_alt * 1000.0, alt * 1000.0, velx, vely, velz, heading);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes)
    Serial.write(buf, len);
}
