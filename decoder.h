#ifndef DECODER_H
#define DECODER_H

#include "Mavlink2/Airside_Functions.hpp"


/* @brief Decoder for mavlink that takes care of byte-by-byte serialization
 *
 * @param encoded_msg Message to be decoded
 * @param decoded_message_buffer Output for decoded message (must be of char[50])
 * @param type Message type according to POGI enum
 *
 * @return status of decoding (0 for success, -1 for failure)
 */

int decode_Mavlink_message (char* decoded_message_buffer, POGI_Message_IDs_e* type, mavlink_message_t encoded_msg);

#endif // DECODER_H
