/**************************************************************************************************/
// Author: Jingting Liu
// April 9th, 2021

// refer to this page for the stucture of mavlink messages 
// https://mavlink.io/en/guide/serialization.html

/**************************************************************************************************/

#ifndef AIRSIDE_FUNCTIONS_HPP
#define AIRSIDE_FUNCTIONS_HPP

#include "Mavlink2_lib/common/mavlink.h"
#include "Encodings.hpp"

//-------------------------- Prototypes ---------------------------------------------------------------

/**
 * This decoder consists of two parts, parser and decoder.
 * The parser handles mavlink messages one byte at a time. 
 * Once the complete packet could be successfully decoded, the decoder would translate the message into telemetry data.
 * 
 * mavlink_decoding_status_t decoderStatus = MAVLINK_DECODING_INCOMPLETE;
    mavlink_global_position_int_t global_position_decoded;

    while(byteavailable == true) // 50 is just a random number larger than message length (for GPS message length is 39)
    {
        // get incoming byte here, decoder can pick up the starting byte by itself

        if (decoderStatus != MAVLINK_DECODING_OKAY) //this make sure it stops after a whole message is received
        {
            decoderStatus = Mavlink_airside_decoder(MAVLINK_COMM_0, current_byte, (uint8_t*) &global_position_decoded);
        }
    }
 * 
 * 
    // the built in GPS struct and its associated encoder and decoder is used for customized warg commands:
    uint32_t warg_ID = global_position.time_boot_ms; // This is used as the WARG message ID
    int32_t latitude = global_position.lat; //< [degE7] Latitude, expressed
    int32_t longitude = global_position.lon; //< [degE7] Longitude, expressed
    int32_t altitude = global_position.alt; //< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
    int32_t relative_altitude = global_position.relative_alt; //< [mm] Altitude above ground
    int16_t Vx = global_position.vx; //< [cm/s] Ground X Speed (Latitude, positive north)
    int16_t Vy = global_position.vy; //< [cm/s] Ground Y Speed (Longitude, positive east)
    int16_t Vz = global_position.vz; //< [cm/s] Ground Z Speed (Altitude, positive down)
    uint16_t Hdg = global_position.hdg; // This carries simple commands such as bool or uint8_t

 **/
mavlink_decoding_status_t Mavlink_airside_decoder(PIGO_Message_IDs_e* type, uint8_t incomingByte, uint8_t *telemetryData);

/**
 * @brief Encode a selected struct
 *
 * @param type The type of telemetry data e.g. GPS or takeoff or landing...
 * @param message The MAVLink message to compress the data into, this is guaranteed to be a full message starts from byte 0xfd
 * @param struct_ptr C-struct to read the message contents from
 * 
 * @return the status of encoding
 * 
 * Example usage, give the address of the struct you want to encode
 *     mavlink_global_position_int_t global_position = 
    {
        ...
    };

    mavlink_message_t encoded_msg;

    uint8_t encoderStatus = Mavlink_airside_encoder(MESSAGE_ID_GPS, &encoded_msg, (const uint8_t*) &global_position);
 */
mavlink_encoding_status_t Mavlink_airside_encoder(POGI_Message_IDs_e id, mavlink_message_t *message, const uint8_t *struct_ptr);

int test__encode_then_decode(void);


#endif //AIRSIDE_FUNCTIONS_HPP

