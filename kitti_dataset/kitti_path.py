import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.transform import Rotation as R


class KITTIPathPublisher(Node):
    def __init__(self):
        super().__init__("kitti_path_publisher")
        self.publisher_ = self.create_publisher(Path, "/gt_path", 10)
        self.path = Path()
        self.path.header = Header(frame_id="map")
        self.load_kitti_data(
            "/home/kdw/dataset/data_odometry_gray/dataset/sequences/00/KITTI_00_gt.txt"
        )
        # self.timer = self.create_timer(
        #     1.0, self.publish_path
        # )  # 매초 path 데이터를 게시합니다.

        self.publish_path()

    def load_kitti_data(self, file_path):
        with open(file_path, "r") as file:
            for line_number, line in enumerate(file):
                data = list(map(float, line.strip().split()))
                rotation_matrix = np.array(
                    [
                        data[0],
                        data[1],
                        data[2],
                        data[4],
                        data[5],
                        data[6],
                        data[8],
                        data[9],
                        data[10],
                    ]
                ).reshape(3, 3)
                translation_vector = np.array([data[3], data[7], data[11]])

                pose = PoseStamped()
                pose.header = Header(
                    frame_id="map", stamp=self.get_clock().now().to_msg()
                )
                pose.pose.position = Point(
                    x=translation_vector[2],
                    y=translation_vector[0] * -1,
                    z=translation_vector[1],
                )
                r = R.from_matrix(rotation_matrix)
                quat = r.as_quat()  # Convert rotation matrix to quaternion [x, y, z, w]
                pose.pose.orientation = Quaternion(
                    x=quat[0], y=quat[1], z=quat[2], w=quat[3]
                )
                self.path.poses.append(pose)

    def publish_path(self):
        self.publisher_.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    kitti_path_publisher = KITTIPathPublisher()

    try:
        rclpy.spin(kitti_path_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        kitti_path_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
