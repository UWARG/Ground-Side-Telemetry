#include <iostream>
#include <QDebug>

#include "decoder.h"
#include "Mavlink2/Mavlink2_lib/common/common.h"
#include "Mavlink2/Airside_Functions.hpp"

using namespace std;

int decode_Mavlink_message (char* decoded_message_buffer, POGI_Message_IDs_e* type, mavlink_message_t encoded_msg) {

    mavlink_decoding_status_t decoderStatus = MAVLINK_DECODING_INCOMPLETE;

    // the following few lines imitates how a decoder is used when it gets one byte at a time from a serial port
    unsigned char* ptr_in_byte = (unsigned char *) &encoded_msg;
    *type = POGI_MESSAGE_ID_NONE;

    for( int i = 0; i < 50; i++) // 50 is just a random number larger than message length (for GPS message length is 39)
    {
        if (decoderStatus != MAVLINK_DECODING_OKAY)
        {
            printf("copying byte: %d  |  current byte : %hhx\n", i, ptr_in_byte[i]);
            decoderStatus = Mavlink_groundside_decoder(type, ptr_in_byte[i], (uint8_t*) &decoded_message_buffer);
        }
    }

    if (decoderStatus == MAVLINK_DECODING_OKAY)
    {
        /* success case */
        return 0;
    }
    else
    {
        /* failure case */
        cerr << "Failed to decode Mavlink message. Continuing as if nothing ever happened" << endl;
        return -1;
    }
}
