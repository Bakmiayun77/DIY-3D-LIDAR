#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, JointState, PointCloud2
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm

class Lidar3DMapper:
    def __init__(self):
        rospy.init_node('lidar_3d_mapper')

        # Gunakan LaserProjection untuk konversi scan ke point cloud
        self.laser_projector = LaserProjection()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.current_angle = 0.0
        self.lidar_frame = rospy.get_param('~lidar_frame', 'lidar_link')
        self.fixed_frame = rospy.get_param('~fixed_frame', 'base_link')

        rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.pub = rospy.Publisher("/pointcloud_3d", PointCloud2, queue_size=10)

    def joint_callback(self, msg):
        # Pastikan joint index ditemukan
        try:
            idx = msg.name.index('rotation_joint')
            self.current_angle = msg.position[idx]
        except ValueError:
            pass

    def scan_callback(self, scan_msg):
        try:
            # Convert LaserScan → PointCloud2 di frame lidar
            pc2_msg = self.laser_projector.projectLaser(scan_msg)

            # Transform ke fixed_frame, termasuk translasi dan rotasi
            trans = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                pc2_msg.header.frame_id,
                scan_msg.header.stamp,
                rospy.Duration(1.0)
            )
            pc2_msg = tf2_sm.do_transform_cloud(pc2_msg, trans)

            # Extract points ke numpy array (Nx3)
            points = np.array(list(
                pc2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True)
            ))

            if points.size == 0:
                return

            # Bangun matriks rotasi di sumbu Y dengan current_angle
            c, s = np.cos(self.current_angle), np.sin(self.current_angle)
            R = np.array([[1, 0, 0],
                          [0, c, -s],
                          [0, s,  c]])

            # Terapkan rotasi secara vektorial
            rotated = points.dot(R.T)

            # Jika ada offset translasi antara joint dan frame lidar
            # Dapatkan transform joint ke lidar jika diperlukan
            # >>> Optional: tambahkan translasi pada rotated points

            # Publish PointCloud2 baru di fixed_frame
            header = pc2_msg.header
            header.frame_id = self.fixed_frame
            cloud_out = pc2.create_cloud_xyz32(header, rotated)
            self.pub.publish(cloud_out)

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup gagal: {e}")
        except Exception as e:
            rospy.logerr(f"Gagal memproses scan: {e}")

if __name__ == "__main__":
    try:
        mapper = Lidar3DMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

