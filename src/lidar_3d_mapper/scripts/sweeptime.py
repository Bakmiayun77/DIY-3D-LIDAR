#!/usr/bin/env python
import rospy
import serial
import os
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
from datetime import datetime

def map_angle_rad_to_center(angle_rad, center_rad=math.pi):
    diff = angle_rad - center_rad
    diff = (diff + math.pi) % (2 * math.pi) - math.pi
    return diff

class ServoSpeedLogger:
    def __init__(self):
        rospy.init_node('servo_speed_logger')
        self.serial_port = rospy.get_param('~port', '/dev/ttyUSB2')
        self.baud_rate = rospy.get_param('~baudrate', 115200)
        self.joint_name = rospy.get_param('~joint_name', 'Bearing')
        # Batas tepat servo
        self.start_angle = rospy.get_param('~start_angle', 2.881)
        self.end_angle   = rospy.get_param('~end_angle',   3.401)
        self.tolerance   = rospy.get_param('~tolerance', 0.005)
        
        default_log = os.path.expanduser('~/servo_speed_log.csv')
        self.log_file = rospy.get_param('~log_file', default_log)
        rospy.loginfo(f"Logging to: {self.log_file}")
        rospy.loginfo(f"Threshold start={self.start_angle}, end={self.end_angle}, tol={self.tolerance}")
        
        self.state = 'idle'
        self.start_time = None
        self.start_angle_record = None
        self.last_angle = None

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            rospy.loginfo(f"Connected to {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to {self.serial_port}: {e}")
            rospy.signal_shutdown("Serial connection failed")
            return

        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        if not os.path.exists(self.log_file):
            with open(self.log_file, 'w') as f:
                f.write('time_start_iso,time_end_iso,angle_start_rad,angle_end_rad,duration_s,avg_speed_rad_per_s,direction\n')
        self.rate = rospy.Rate(rospy.get_param('~hz', 1000))

    def append_log(self, t_start_ros, t_end_ros, angle_start, angle_end, duration, avg_speed, direction):
        # Konversi rospy.Time ke datetime (UTC)
        try:
            ts = t_start_ros.to_sec()
            te = t_end_ros.to_sec()
            dt_start = datetime.utcfromtimestamp(ts)
            dt_end   = datetime.utcfromtimestamp(te)
            timestr_start = dt_start.isoformat()
            timestr_end   = dt_end.isoformat()
        except Exception:
            timestr_start = str(t_start_ros.to_sec())
            timestr_end   = str(t_end_ros.to_sec())
        # Gunakan {} untuk timestamp (string), {:.6f} untuk angka
        with open(self.log_file, 'a') as f:
            line = "{},{},{:.6f},{:.6f},{:.6f},{:.6f},{}\n".format(
                timestr_start,   # string
                timestr_end,     # string
                angle_start,     # float
                angle_end,       # float
                duration,        # float
                avg_speed,       # float
                direction        # string
            )
            f.write(line)
        rospy.loginfo(f"Logged {direction}: dur {duration:.3f}s, speed {avg_speed:.3f} rad/s")

    def process_angle(self, angle_rad):
        tol = self.tolerance
        sa = self.start_angle
        ea = self.end_angle
        if self.last_angle is None:
            self.last_angle = angle_rad
            return
        la = self.last_angle
        ca = angle_rad

        if self.state == 'idle':
            # start forward: di batas start dan naik
            if abs(la - sa) < tol and ca > la:
                self.state = 'measuring_forward'
                self.start_time = rospy.Time.now()
                self.start_angle_record = la
                rospy.loginfo(f"Start measuring FORWARD at ~{la:.3f}")
            # start reverse: di batas end dan turun
            elif abs(la - ea) < tol and ca < la:
                self.state = 'measuring_reverse'
                self.start_time = rospy.Time.now()
                self.start_angle_record = la
                rospy.loginfo(f"Start measuring REVERSE at ~{la:.3f}")

        elif self.state == 'measuring_forward':
            if abs(ca - ea) < tol:
                end_time = rospy.Time.now()
                a0 = self.start_angle_record
                a1 = ea
                duration = (end_time - self.start_time).to_sec()
                delta = abs(a1 - a0)
                avg_speed = delta / duration if duration > 0 else float('inf')
                self.append_log(self.start_time, end_time, a0, a1, duration, avg_speed, 'forward')
                self.state = 'idle'
                self.start_time = None
                self.start_angle_record = None

        elif self.state == 'measuring_reverse':
            if abs(ca - sa) < tol:
                end_time = rospy.Time.now()
                a0 = self.start_angle_record
                a1 = sa
                duration = (end_time - self.start_time).to_sec()
                delta = abs(a0 - a1)
                avg_speed = delta / duration if duration > 0 else float('inf')
                self.append_log(self.start_time, end_time, a0, a1, duration, avg_speed, 'reverse')
                self.state = 'idle'
                self.start_time = None
                self.start_angle_record = None

        self.last_angle = angle_rad

    def run(self):
        while not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if not line:
                    self.rate.sleep()
                    continue
                try:
                    angle_rad = float(line)
                except ValueError:
                    rospy.logwarn(f"Bad line: '{line}'")
                    self.rate.sleep()
                    continue

                # Publish JointState
                centered = map_angle_rad_to_center(angle_rad, center_rad=math.pi)
                js = JointState()
                js.header = Header()
                js.header.stamp = rospy.Time.now()
                js.name = [self.joint_name]
                js.position = [centered]
                self.pub.publish(js)

                # Proses speed
                self.process_angle(angle_rad)

            except Exception as e:
                rospy.logwarn(f"Main loop error: {e}")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ServoSpeedLogger()
        node.run()
    except rospy.ROSInterruptException:
        pass

