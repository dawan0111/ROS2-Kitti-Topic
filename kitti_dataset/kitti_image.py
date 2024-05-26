import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import glob


class ImagePublisher(Node):
    def __init__(self, folders):
        super().__init__("image_publisher")
        self.bridge = CvBridge()
        self.img_publishers = {}
        self.images = {}
        self.index = 0
        self.timer = self.create_timer(1 / 30, self.timer_callback)  # 30 FPS
        names = ["left", "right"]
        # 각 폴더에 대한 퍼블리셔 설정 및 이미지 리스트 로드
        for i, folder in enumerate(folders):
            topic_name = "stereo/image_{0}".format(names[i])
            self.img_publishers[folder] = self.create_publisher(Image, topic_name, 10)
            self.images[folder] = sorted(glob.glob(f"{folder}/*.png"))

        # 이미지 갯수 체크, 최소 값을 공통 길이로 사용
        min_length = min(len(self.images[folder]) for folder in folders)
        for folder in folders:
            self.images[folder] = self.images[folder][:min_length]

        self.timer_callback()

    def timer_callback(self):
        # input()
        if self.index < len(next(iter(self.images.values()))):
            for folder, publisher in self.img_publishers.items():
                image_path = self.images[folder][self.index]
                cv_image = cv2.imread(image_path)
                if cv_image is not None:
                    ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                    publisher.publish(ros_image)
                    self.get_logger().info(
                        f"Publishing: {image_path} on {publisher.topic_name}"
                    )
            self.index += 1
        else:
            self.get_logger().info("Finished publishing all images.")
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    folders = [
        "/home/kdw/dataset/data_odometry_gray/dataset/sequences/00/image_0",
        "/home/kdw/dataset/data_odometry_gray/dataset/sequences/00/image_1",
    ]
    image_publisher = ImagePublisher(folders)
    rclpy.spin(image_publisher)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
