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

// This should literally just counteract the yaw stabilization of the gimbal because we cant (??) disable it >:)

int main() {

    int pin0 = 23;
    int pin1 = 24;

    if (gpioInitialise() < 0) {
        std::cout << "Initialization failed uWu" << std::endl;
	exit(1);
    }

    gpioSetMode(pin0, PI_OUTPUT);
    int value0 = 1500;
    gpioServo(pin0,value0);
    
    gpioSetMode(pin1, PI_OUTPUT);
    int value1 = 1500;
    gpioServo(pin1,value1);

	int servo = 0;
	int v = 0;

	for(;;){

		std::cout << "\nSelect servo (0 or 1): ";
		std::cin >> servo;
		if (servo != 0) {
			servo = 1;
		}
		
		std::cout << "\nSelect input (-100 to 100): ";
		std::cin >> v;
		
		if (servo == 0) {
			value0 = 1500 + (v * 5);
			gpioServo(pin0, value0); 
		} else {
			value1 = 1500 + (v * 5);
			gpioServo(pin1, value1);
		}
	
	}

}
