#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import csv
import time

class LidarScanLogger:
    def __init__(self):
        self.last_time = None
        self.csv_file = open("/home/jeffry/scan_rate_log.csv", "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["ROS Timestamp", "Time Delta (s)", "Scan Rate (Hz)"])
        rospy.Subscriber("/scan", LaserScan, self.callback)
        rospy.loginfo("Started logging scan rate to CSV.")

    def callback(self, data):
        current_time = data.header.stamp.to_sec()
        if self.last_time is not None:
            delta = current_time - self.last_time
            rate = 1.0 / delta if delta > 0 else 0
            rospy.loginfo(f"Scan rate: {rate:.2f} Hz")
            self.writer.writerow([current_time, delta, rate])
        self.last_time = current_time

    def run(self):
        rospy.spin()
        self.csv_file.close()

if __name__ == "__main__":
    rospy.init_node("lidar_scanrate_logger", anonymous=True)
    logger = LidarScanLogger()
    logger.run()
