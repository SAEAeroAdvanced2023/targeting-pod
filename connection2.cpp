//
// from: https://www.codeproject.com/Questions/1000793/Use-serial-port-object-in-Cplusplus-file
#include "..\c_library_v2\common\mavlink.h"

// from: https://mavlink.io/en/mavgen_c/
mavlink_system_t mavlink_system = {
        1, // System ID (1-255)
        1  // Component ID (a MAV_COMPONENT value)
};

mavlink_status_t status;
mavlink_message_t msg;
int chan = MAVLINK_COMM_0;

while(serial.bytesAvailable > 0)
{
uint8_t byte = serial.getNextByte();
if (mavlink_parse_char(chan, byte, &msg, &status))
{
printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
// ... DECODE THE MESSAGE PAYLOAD HERE ...
}
}

if (mavlink_parse_char(chan, byte, &msg, &status)) {
    switch(msg.msgid) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: { // ID for GLOBAL_POSITION_INT
            // Get all fields in payload (into global_position)
            mavlink_msg_global_position_int_decode(&msg, &global_position);

            }
            break;
        case MAVLINK_MSG_ID_GPS_STATUS:{
            // Get just one field from payload
            visible_sats = mavlink_msg_gps_status_get_satellites_visible(&msg);
            }
            break;
        default:
            break;
}
}