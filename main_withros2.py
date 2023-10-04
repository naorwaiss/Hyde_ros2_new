#doing some test with mavros
from pymavlink import mavutil
import rospy
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
import time, math
from mavros_msgs.msg import Mavlink

# object define
class fifo(object):
    """ A simple buffer """

    def __init__(self):
        self.buf = []

    def write(self, data):
        self.buf += data
        return len(data)

    def read(self):
        return self.buf.pop(0)

# the fake position of the drone
class HydeMotion(object):
    lat = int(32.0747 * 1e7)  # Terni
    lon = int(34.7648 * 1e7)  # Terni
    alt = int(1 * 1e3)

    def setup(self):
        abir = mavutil.set_dialect("ardupilotmega")
        self.autopilot = mavutil.mavlink_connection('udp:127.0.0.1:14550')

        msg = None
        print("before heartbeat")
        self.autopilot.wait_heartbeat()

        print("before rcv msg")
        # wait for autopilot connection
        while msg is None:
            msg = self.autopilot.recv_msg()

        self.mavlink_pub = rospy.Publisher("/mavlink/to", Mavlink, queue_size=20)
        # Set up mavlink instance
        f = fifo()
        self.mav = MAV_APM.MAVLink(f, srcSystem=1, srcComponent=1)

        # wait to initialize
        while self.mavlink_pub.get_num_connections() <= 0:
            pass

    def arm(self):
        print("arming")
        time.sleep(1)
        self.autopilot.mav.command_long_send(
            0,  # autopilot system id
            0,  # autopilot component id
            400,  # command id, ARM/DISARM
            0,  # confirmation
            1,  # arm!
            0, 0, 0, 0, 0, 0)  # unused parameters for this command
        time.sleep(4)

def main():
    hyde_motion = HydeMotion()
    hyde_motion.setup()
    hyde_motion.arm()

if __name__ == "__main__":
    main()
