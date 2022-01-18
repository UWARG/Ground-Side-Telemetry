/**************************************************************************************************/
// Author: Jingting Liu
// April 9th, 2021

// refer to this page for the stucture of mavlink messages 
// https://mavlink.io/en/guide/serialization.html

/**************************************************************************************************/

#ifndef ENCODINGS_HPP
#define ENCODINGS_HPP

#include "Mavlink2_lib/common/mavlink.h"

enum mavlink_decoding_status_t{
    DECODING_INCOMPLETE=0,
    DECODING_OKAY=1,
    DECODING_FAIL=2,
};

enum mavlink_encoding_status_t{
    MAVLINK_ENCODING_INCOMPLETE=0,
    MAVLINK_ENCODING_OKAY=1,
    MAVLINK_ENCODING_BAD_ID = 2,
    MAVLINK_ENCODING_FAIL=3,
};

//airside decoder, grounside encoder, Plane In Ground Out (PIGO)
enum PIGO_Message_IDs_e{
    MESSAGE_ID_NONE = 0,
    MESSAGE_ID_GPS_LANDING_SPOT,
    MESSAGE_ID_GROUND_CMD,
    MESSAGE_ID_GIMBAL_CMD,
    MESSAGE_ID_BEGIN_LANDING,
    MESSAGE_ID_BEGIN_TAKEOFF,
    MESSAGE_ID_NUM_WAYPOINTS,
    MESSAGE_ID_WAYPOINT_MODIFY_PATH_CMD,
    MESSAGE_ID_WAYPOINT_NEXT_DIRECTIONS_CMD,
    MESSAGE_ID_INITIALIZING_HOMEBASE,
    MESSAGE_ID_HOLDING_ALTITUDE,
    MESSAGE_ID_HOLDING_TURN_RADIUS,
    MESSAGE_ID_HOLDING_TURN_DIRECTION,
    MESSAGE_ID_PATH_MODIFY_NEXT_LD,
    MESSAGE_ID_PATH_MODIFY_PREV_LD,
    MESSAGE_ID_PATH_MODIFY_LD,
    MESSAGE_ID_WAYPOINTS,
    MESSAGE_ID_HOMEBASE,
};

//airside encoder, groundside decoder, Plane Out Ground In (POGI)
enum POGI_Message_IDs_e{
    POGI_MESSAGE_ID_NONE = -1,
    MESSAGE_ID_TIMESTAMP = 0,
    MESSAGE_ID_GPS,
    MESSAGE_ID_ERROR_CODE,
    MESSAGE_ID_AIR_SPEED,
    MESSAGE_ID_EULER_ANGLE_PLANE,
    MESSAGE_ID_EULER_ANGLE_CAM,
    MESSAGE_ID_IS_LANDED,
    MESSAGE_ID_EDITING_FLIGHT_PATH_ERROR_CODE,
    MESSAGE_ID_FLIGHT_PATH_FOLLOWING_ERROR_CODE,
    MESSAGE_ID_CURRENT_WAYPOINT_LD,
    MESSAGE_ID_CURRENT_WAYPOINT_INDEX,
    MESSAGE_ID_HOMEBASE_INITIALIZED,
};

//-------------------------- Customized WARG Command Structs ---------------------------------------------------------------

struct PIGO_GPS_LANDING_SPOT_t {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    int32_t landingDirection;
};

struct PIGO_WAYPOINTS_t { // same as homebase GPS struct
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    int32_t turnRadius;
    uint8_t waypointType;
};

struct PIGO_GIMBAL_t { // convert to float
    int32_t pitch;
    int32_t yaw;
};

struct PIGO_GROUND_COMMAND_t { //convert to float
    int32_t heading;
    int32_t latestDistance;
};


struct single_bool_cmd_t {
    bool cmd;
};

struct one_byte_uint_cmd_t {
    uint8_t cmd;
};

struct four_bytes_int_cmd_t {
    int32_t cmd;
};

struct four_bytes_float_cmd_t {
    float cmd;
};

struct POGI_Euler_Angle_t { // convert to float
    int32_t yaw;
    int32_t pitch;
    int32_t roll;
};

struct POGI_GPS_t {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
};

struct POGI_Timestamp_t {
    uint32_t timeStamp;
};

#endif  // ENCODINGS_HPP
