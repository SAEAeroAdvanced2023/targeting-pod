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
        exit(1);
    }

    std::cout << "Serial port opened" << std::endl;

    int buffer = 99;

    uint8_t message[buffer];

    const char command[] = {'\x3E', '\x52', '\x01', '\x53', '\x00', '\x00'};
    //const char command[] = {'\x3E', '\x44', '\x00', '\x44', '\x00'};

    while(true) {

        for (char i: command) {
            write(serial_port, &i, sizeof(i));
        }

        std::cout << "Command Written" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        read(serial_port, &message, sizeof(message));

        std::cout << "Message received" << std::endl;

        int header_checksum = message[1] + message[2];
        if (header_checksum % 256 != message[3]){
            std::cout << "Header checksum invalid: " << (int) header_checksum % 256 << " vs " << (int) message[3] << std::endl;
            exit(1);
        }
        int body_checksum = 0;
        for (int i = 0; i < (int) message[2]; i++){
            body_checksum += (int) message[i+4];
        }
        if (body_checksum % 256 != message[message[2]+4]){
            std::cout << "Body checksum invalid: " << (int) body_checksum % 256 << " vs " << (int) message[2] << std::endl;
            exit(1);
        }

//        for (int i = 0; i < buffer; i++) {
//            printf("%d: %x\n", i, message[i]);
//        }
//
//        std::cout << (unsigned int) message[2] << std::endl;

        std::cout << "Checksum passed" << std::endl;

        for (int i = 0; i < buffer - 1; i++) {
            int16_t bb = (message[i+1] << 8) | (message[i]);
            std::cout << i << ": " << bb << std::endl;
        }

        int16_t t = (message[26] << 8) | (message[25]);
        std::cout << "Received Parameter: " << t << std::endl;

        message[1] = '\x57';
        message[3] = message[1] + message[2];

        message[25] = 0b01001100;
        message[26] = 0b11111111;
        message[27] = 0b10110100;
        message[28] = 0b00000000;

        message[33] = 0b01001100;
        message[34] = 0b11111111;
        message[35] = 0b10110100;
        message[36] = 0b00000000;

        message[41] = 0b01001100;
        message[42] = 0b11111111;
        message[43] = 0b10110100;
        message[44] = 0b00000000;

        int q = 0;
        for (int i = 4; i < buffer - 1; i++) {
            q += (int) message[i];
        }
        message[98] = (char) q % 256;

        std::cout << "Message Edited" << std::endl;

        header_checksum = message[1] + message[2];
        if (header_checksum % 256 != message[3]){
            std::cout << "Header checksum invalid: " << (int) header_checksum % 256 << " vs " << (int) message[3] << std::endl;
            exit(1);
        }
        body_checksum = 0;
        for (int i = 0; i < (int) message[2]; i++){
            body_checksum += (int) message[i+4];
        }
        if (body_checksum % 256 != message[message[2]+4]){
            std::cout << "Body checksum invalid: " << (int) body_checksum % 256 << " vs " << (int) message[2] << std::endl;
            exit(1);
        }

        std::cout << "Edited message checksum passed" << std::endl;

        int16_t w = (message[26] << 8) | (message[25]);
        std::cout << "New Parameter: " << w << std::endl;

        for (char i: message) {
            write(serial_port, &i, sizeof(i));
        }

        std::cout << "Edited message sent" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        read(serial_port, &message, sizeof(message));

        std::cout << "Message received" << std::endl;

        header_checksum = message[1] + message[2];
        if (header_checksum % 256 != message[3]){
            std::cout << "Header checksum invalid: " << (int) header_checksum % 256 << " vs " << (int) message[3] << std::endl;
            exit(1);
        }
        body_checksum = 0;
        for (int i = 0; i < (int) message[2]; i++){
            body_checksum += (int) message[i+4];
        }
        if (body_checksum % 256 != message[message[2]+4]){
            std::cout << "Body checksum invalid: " << (int) body_checksum % 256 << " vs " << (int) message[2] << std::endl;
            exit(1);
        }

        std::cout << "Checksum passed" << std::endl;

        int16_t b = (message[26] << 8) | (message[25]);
        std::cout << "Updated Parameter: " << b << std::endl;

        break;

    }

}
// 25!!!