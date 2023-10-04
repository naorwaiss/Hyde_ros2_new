import time
from pymavlink import mavutil
import asyncio

class HydeMotion(object):
    lat = int(32.0747 * 1e7)  # Terni
    lon = int(34.7648 * 1e7)  # Terni
    alt = int(1 * 1e3)
    arm = 0  # 0 is safe, 1 is armed
    mode = None

    def __init__(self):
        self.setup()

    def setup(self):
        # Set up MAVLink connection
        abir = mavutil.set_dialect("ardupilotmega")
        self.autopilot = mavutil.mavlink_connection('udp:127.0.0.1:14550')

        print("Before heartbeat")
        self.autopilot.wait_heartbeat()

    def arm(self):
        # Arm the drone
        self.autopilot.mav.command_long_send(
            self.autopilot.target_system,
            self.autopilot.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0, 0, 0, 0, 0, 0  # Not using parameters
        )
        msg = self.autopilot.recv_match(type='COMMAND_ACK', blocking=True)
        if msg.result == 0:
            print("The Hyde is armed.")
            self.arm = 1

    def start_guided(self):
        # Change to GUIDED mode:
        print("GUIDED mode")
        #self.set_global_origin()
        #self.set_home_position()

        mavutil.mavfile.set_mode(self.autopilot, 4, 0, 0)
        self.mode = "Guided"
        time.sleep(1)
        # self.gyro_calibration() -- not defined for now
        self.arm()
        self.arm = 1

    async def takeoff(self, takeoff_alt):
        await asyncio.sleep(0)  # Placeholder for awaiting something

        # Check if the drone is armed and in guided mode (replace with your own check)
        if self.arm == 1 and self.mode == "Guided":
            print("Takeoff to " + str(takeoff_alt) + " meters height")

            # Send the takeoff command
            self.autopilot.mav.command_long_send(
                0,  # autopilot system id
                0,  # autopilot component id
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command id, MAV_CMD_NAV_TAKEOFF
                0,  # confirmation
                0,  #
                0, 0, 0, 0, 0, takeoff_alt)  # height

            await asyncio.sleep(6 * takeoff_alt)
            msg = self.autopilot.recv_match(type='COMMAND_ACK', blocking=True)
            if msg.result == 4:
                print("Command accepted")
                print("Finished takeoff")







async def main():
    hyde_motion = HydeMotion()

    while True:
        command = input("Enter a command (guided/takeoff/exit): ")
        if command == "guided":
            hyde_motion.start_guided()
        elif command == "takeoff":
            await hyde_motion.takeoff(5)  # Remove await here
            print("Task finished")
        elif command == "exit":
            break
        else:
            print("Invalid command. Available commands: guided, takeoff, exit")


if __name__ == "__main__":
    asyncio.run(main())
