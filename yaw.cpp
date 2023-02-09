#include <iostream>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <pigpio.h>

void func(){

}

int main() {

    uint8_t message[60];
    int pin = 23;
    int imu_port = open("/dev/ttyACM0", O_RDWR);
    if (imu_port < 0) {
        std::cout << "Erororrr :( :(" << std::endl;
        exit(1);
    }

    if (gpioInitialise() < 0) {
        std::cout << "Initializtion failed uwu" << std::endl;
    }

    gpioSetMode(pin, PI_OUTPUT);

    int value = 1500;
    gpioServo(pin,value);

    const char command[] = {'\x3E', '\x44', '\x00', '\x44', '\x00'}; // Command to request data

    int header_checksum = 0;
    int body_checksum = 0;

    for (;;) { // Cool infinite loop notation >:)

        for (int i = 0; i < 5; i++) { // length of command
            write(imu_port, &command[i], sizeof(command[i]));
        }

        // Wait for response from the gimbal board
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // DO NOT GO LOWER THAN 35 PLEASE OR PLANE EXPLODES

        read(imu_port, &message, sizeof(message));

        header_checksum = message[1] + message[2];
        if (header_checksum % 256 != message[3]) {
            std::cout << "Header checksum invalid" << std::endl;
            continue;
        }
        body_checksum = 0;
        for (int i = 0; i < (int) message[2]; i++) {
            body_checksum += (int) message[i+4];
        }
        if (body_checksum % 256 != message[message[2] + 4]) {
            std::cout << "Body checksum invalid" << std::endl;
            continue;
        }

        int16_t yaw = (message[47] << 8) | (message[46]);
        yaw /= 10.0;



    }

}
