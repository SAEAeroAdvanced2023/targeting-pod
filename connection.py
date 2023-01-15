from pymavlink import mavutil
import time

def main():
    print("Attempting connection")
    vehicle = mavutil.mavlink_connection(device="/dev/ttyACM0", baud="57600")
    print("Waiting for heartbeat...")
    vehicle.wait_heartbeat()
    print("Success!!!")
    print("Setting up receivers...")
    # Message codes:
    # GPS_RAW_INT: 24
    # GPS_STATUS: 25
    # SCALED_IMU: 26
    # ATTITUDE: 30
    # GLOBAL_POSITION_INT 33
    vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            30, # Message code goes here
            1000000, # Inteval in microseconds
            0,
            0,
            0,
            0,
            0)
    print("Receivers configured")
    data = "ATTITUDE"
    while(True):
        vehicle.recv_match(type=data, blocking=True)
        x = vehicle.messages[data].roll
        y = vehicle.messages[data].pitch
        z = vehicle.messages[data].yaw
        time = vehicle.messages[data].time_boot_ms # usually time_boot_ms or time_usec
        print("[" + str(time) + "] roll: " + str(x) + " pitch: " + str(y) + " yaw: " + str(z))

if __name__ == "__main__":
    main()
    
