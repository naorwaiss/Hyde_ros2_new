import time
from pymavlink import mavutil

# Connect to the autopilot via MAVLink (adjust the serial port as needed)
autopilot = mavutil.mavlink_connection('/dev/ttyTHS0', baud=57600)

# Set the global origin (you can adjust the latitude, longitude, and altitude)
latitude = 32.0747 * 1e7  # Terni
longitude = 34.7648 * 1e7  # Terni
altitude = 0  # Adjust as needed

# Send a SET_GPS_GLOBAL_ORIGIN message to set the origin
autopilot.mav.set_gps_global_origin_send(
    latitude, longitude, altitude
)

while True:
    msg = autopilot.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg is not None:
        latitude = msg.lat / 1e7  # Convert latitude to degrees
        longitude = msg.lon / 1e7  # Convert longitude to degrees
        altitude = msg.alt / 1e3  # Convert altitude to meters

        print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude} meters")
