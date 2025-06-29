#!/usr/bin/env python
import rospy
import serial
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

def map_angle_rad_to_center(angle_rad, center_rad=math.pi):
    """
    Shift the input angle range so that 'center_rad' maps to 0 rad.
    Misal, jika Arduino mengirim 3.14 rad untuk 180°, maka hasilnya 0.
    Fungsi ini juga menormalisasi ke [-π, π) agar tetap dalam range wajar.
    """
    diff = angle_rad - center_rad
    # Normalisasi ke [-π, π)
    diff = (diff + math.pi) % (2 * math.pi) - math.pi
    return -diff

def main():
    rospy.init_node('servo_joint_publisher')
    
    serial_port = rospy.get_param('~port', '/dev/ttyUSB2')
    baud_rate = rospy.get_param('~baudrate', 115200)
    joint_name = rospy.get_param('~joint_name', 'Bearing')

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        rospy.loginfo(f"Connected to {serial_port} at {baud_rate} baud.")
    except serial.SerialException:
        rospy.logerr(f"Failed to connect to {serial_port}")
        return

    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(1000)  # loop up to 1000 Hz, sesuaikan kebutuhan baca serial

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                # Asumsikan line adalah sudut dalam radian
                angle_rad = float(line)

                # Offset tengah di π rad (180°)
                centered_rad = map_angle_rad_to_center(angle_rad, center_rad=math.pi)

                # Buat dan publish JointState dengan posisi dalam radian
                joint_state = JointState()
                joint_state.header = Header()
                joint_state.header.stamp = rospy.Time.now()
                joint_state.name = [joint_name]
                joint_state.position = [centered_rad]

                pub.publish(joint_state)
        except Exception as e:
            rospy.logwarn(f"Failed to read/parse serial: {e}")
        
        rate.sleep()

if __name__ == '__main__':
    main()

