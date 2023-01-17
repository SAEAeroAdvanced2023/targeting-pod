//DOES NOT WORK!!!! JUST TESTING OUT STUFF

#include <iostream>
#include <mavlink/v2.0/common/mavlink.h>
#include <SerialPort.h>

using namespace std;

// try using: https://mavlink.io/en/mavgen_c/
// look into: https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS_Common.cpp
mavlink_system_t mavlink_system = {
        1, // System ID (1-255)
        1  // Component ID (a MAV_COMPONENT value)
};

int main() {
    // Initialize serial port
    SerialPort serialPort("/dev/ttyACM0", 57600);
    if (!serialPort.isOpen()) {
        std::cout << "Error opening serial port" << std::endl;
        return 1;
    }

    // MAVLink message variables
    mavlink_message_t message;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len;

    // Send MAVLink ping message
    mavlink_msg_ping_pack(1, 0, &message, 0, 0, 0, 0, 0);
    len = mavlink_msg_to_send_buffer(buffer, &message);
    serialPort.writeData(buffer, len);

    // Receive response
    len = serialPort.readData(buffer, MAVLINK_MAX_PACKET_LEN);
    if (len > 0) {
        mavlink_message_t msg;
        mavlink_status_t status;
        for (int i = 0; i < len; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                std::cout << "Received MAVLink message with ID: " << msg.msgid << std::endl;
            }
        }
    }


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

    return 0;
}