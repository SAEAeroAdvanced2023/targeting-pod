#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <chrono>
#include <thread>


int main(){

    volatile int serial_port = open("/dev/ttyACM0", O_RDWR);
    if (serial_port < 0) {
        std::cout << "Error opening serial port!!!" << std::endl;
    }

    int buffer = 60;

    uint8_t byte;
    uint8_t message[buffer];

    //const char command[] = {'\x3E', '\x52', '\x01', '\x53', '\x01', '\x01'};
    const char command[] = {'\x3E', '\x44', '\x00', '\x44', '\x00'};

    //write(serial_port, command, sizeof(command));

    while(true) {

        for (char i: command) {
            write(serial_port, &i, sizeof(i));
        }

        //std::cout << "Sent Command!\nWaiting for response..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        //sleep(1);

        //for(int i = 0; i < buffer; i++){
            read(serial_port, &message, sizeof(message));
            //std::cout << i << ": ";
            /*if (byte == (uint8_t) '\xa1'){
                //std::cout << std::endl;
                break;
            }*/
            //printf("%x\n", byte);
            //message[i] = byte;
        //}

        /*for (int i = 0; i < buffer; i++){
            message[i] = 0;
        }
        for (int i = 0; i < 4; i++) {
            read(serial_port, &byte, sizeof(byte));
            message[i] = byte;
        }
        for (int i = 4; i < (int) message[2] + 4; i++) {
            read(serial_port, &byte, sizeof(byte));
            message[i] = byte;
        }
        for (int i = 0; i < buffer; i++){
            printf("%d : %x\n",i,message[i]);
        }*/

        int header_checksum = message[1] + message[2];
        if (header_checksum % 256 != message[3]){
            std::cout << "Header checksum invalid: " << (int) header_checksum % 256 << " vs " << (int) message[3] << std::endl;
            continue;
        }
        int body_checksum = 0;
        for (int i = 0; i < (int) message[2]; i++){
            body_checksum += (int) message[i+4];
        }
        if (body_checksum % 256 != message[message[2]+4]){
            std::cout << "Body checksum invalid: " << (int) body_checksum % 256 << " vs " << (int) message[2] << std::endl;
            continue;
        }

        /*sleep(2);

        for(char i : command){
            write(serial_port, &i, sizeof(i));
        }

        std::cout << "Sent Command!\nWaiting for response..." << std::endl;
        sleep(1);

        for(int i = 0; i < buffer; i++){
            read(serial_port, &byte, sizeof(byte));
            //std::cout << i << ": ";
            if (byte == (uint8_t) '\xa1'){
                std::cout << std::endl;
                break;
            }
            //printf("%x\n", byte);
            if (message[i] != byte && i < 53){
                std::cout << i << std::endl;
            }
        }*/


        // Data char #32 (start @ 0) is first bit of ANGLE_ROLL
        uint16_t roll = (message[43] << 8) | (message[42]);
        uint16_t pitch = (message[45] << 8) | (message[44]);
        uint16_t yaw = (message[47] << 8) | (message[46]);
        //printf("%x %x\n", message[42], message[43]);
        //printf("%x\n", roll);

        std::cout << "Roll: " << (int16_t) roll / 10.0 << " Pitch: " << (int16_t) pitch / 10.0 << " Yaw: " << (int16_t) yaw / 10.0 << std::endl;


    }

}
