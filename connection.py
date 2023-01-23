# How CommandLong works: https://mavlink.io/en/messages/common.html#COMMAND_LONG
# - Specific command used: https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL
# Obtaining GPS Data:  https://mavlink.io/en/messages/common.html#GPS_RAW_INT
# Obtaining Attitude Data: https://mavlink.io/en/messages/common.html#ATTITUDE
# mavutil defs: https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py
# Examples:
# - https://snyk.io/advisor/python/pymavlink/functions/pymavlink.mavutil.mavlink_connection
# - https://programtalk.com/python-examples/pymavlink.mavutil.mavlink_connection/

from pymavlink import mavutil
import time

def main():
    print("Attempting connection")
        # TODO : when forcing the code to stop, it has issues running again right away (usually fixed by power cycling the cube)
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
    ## MAV_CMD_SET_MESSAGE_INTERVAL: sends ONE dada signal at a specified time interval
    vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            30, # Getting Attitude Raw Data
            1000000, # The interval between two messages (in microseconds)
            0,0,0,0,0) #Keep zero
    vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            24, # Getting GPS Raw Data
            1000000, # The interval between two messages (in microseconds)
            0,0,0,0,0) #Keep zero
    print("Receivers configured")
    attitude = 'ATTITUDE'
    gps = 'GPS_RAW_INT'
    while(True):
        vehicle.recv_match(type=[attitude, gps], blocking=True)
        roll = vehicle.messages[attitude].roll
        pitch = vehicle.messages[attitude].pitch
        yaw = vehicle.messages[attitude].yaw
        time = vehicle.messages[attitude].time_boot_ms # usually time_boot_ms or time_usec
        lat = vehicle.messages[gps].lat
        lon = vehicle.messages[gps].lon
        alt = vehicle.messages[gps].alt
        print("[" + str(time) + "] roll: " + str(roll) + " pitch: " + str(pitch) + " yaw: " + str(yaw)+ " lattitude: " + str(lat) + " longitude: " + str(lon) + " altitude: " + str(alt))

if __name__ == "__main__":
    main()
    
