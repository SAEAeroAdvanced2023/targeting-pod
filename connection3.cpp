#include <iostream>
#include <mavsdk/mavsdk.h>

using namespace std;
using namespace mavsdk;

int main(){
	
	Mavsdk mavsdk;
	
	ConnectionResult connection_result = mavsdk.add_any_connection("serial:///dev/ttyACM0");
	
	if (connection_result != ConnectionResult::Success) {
		std::cout << "Adding connection failed: " << connection_result << '\n';
		return 1;
	}
	
}
