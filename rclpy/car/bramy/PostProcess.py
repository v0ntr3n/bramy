import time

import cv2
import numpy as np
import onnxruntime as ort
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import rclpy
from bramy.ocsort.ocsort import OCSort
from rclpy.node import Node

ocSort = OCSort()

        
class PostProcessing(Node):
    def __init__(self):
        super().__init__('post_prcesss')

        self.create_subscription(Float32MultiArray, 'cordinates', self.detect, 1)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'get_control', 1)
        self.bridge = CvBridge()
        self.trackerID = None
        self.TrackerPos = None
        self.create_subscription(
            Image,
            'depth_image',
            self._depth_callback,
            1  # QoS: queue size
        )
        self._depth_image = None

    def _depth_callback(self, msg):
        """
        Callback for the depth image topic.
        Converts the ROS Image message to OpenCV format (passthrough).
        """
        try:
            # Use "passthrough" to retain the original format of the depth image (e.g., 16UC1, 32FC1)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
            self._depth_image = cv2.flip(cv_image, 1)

        except Exception as e:
            self.get_logger().error(f'Error converting Depth image: {e}')
            self._depth_image = None

    def getCenterBox(self, bboxes):
        tl = 1
        selecting = False
        if self.trackerID is None:
            selecting = True
            centerX = 640 // 2
            centerY = 480 // 2
            maxDist = float('inf')


        for (x1, y1, x2, y2, pos_id) in bboxes:
            if int(pos_id) == self.trackerID:
                self.TrackerPos = x1, x2, y1, y2
                color = (255, 0, 255)
                # self.get_logger().info(color)
            else:
                color = (0, 255, 0)

            c1, c2 = (x1, y1), (x2, y2)
            # cv2.rectangle(image, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
            if selecting:
                x_center = (x1 + x2) // 2
                y_center = (y1 + y2) // 2
                dist = ((centerX - x_center) ** 2 + (centerY - y_center) ** 2)**0.5
                self.get_logger().info(f'{pos_id}, {dist}')
                if dist < maxDist:
                    self.trackerID = int(pos_id)
                    self.TrackerPos = x1, x2, y1, y2
                    maxDist = dist


            # cv2.putText(image, '{} ID-{}'.format(cls_id, pos_id), (c1[0], c1[1] - 2), 0, 1,
            #             [225, 255, 255], thickness=1, lineType=cv2.LINE_AA)

    def FindDistane(self, bboxes):
        if self.TrackerPos:
            for x1, y1, x2, y2, cls_id, depth_value in bboxes:
                self.get_logger().info(f"{self.TrackerPos} {x1}, {x2}, {y1}, {y2}")
                if self.TrackerPos == (x1, x2, y1, y2):
                    return depth_value

        return None

    def detect(self, cordinates):
        bboxes = cordinates.data
        
        if len(bboxes) == 0:
            self.trackerID = None
            self.TrackerPos = None
        else:
            num_detections = len(bboxes) // 5
            bboxes = np.array(bboxes, dtype=np.float32).reshape((num_detections, 5))
            # xywhs = torch.tensor(bboxes)
            depth_value = None
            self.get_logger().info(f"{bboxes.shape}")
            if bboxes.shape[0] == 1:
                multi = False
                if bboxes[0][-2] < 0.4:
                    return
                else:
                    x1, y1, x2, y2, conf = bboxes[0]
                    self.TrackerPos = x1, x2, y1, y2
            else:
                multi = True
                outputs = ocSort.update(bboxes, (640,480), (640,480))
                self.getCenterBox(outputs)


            if self.TrackerPos is not None or (multi and self.TrackerPos):
                x1, x2, y1, y2 = self.TrackerPos


                c1, c2 = int((y1 + y2) // 2), int((x1+ x2) // 2)
                depth_value = self._depth_image[c1, c2]
                threshold_depth = 1000
                
                xc = ((self.TrackerPos[0] + self.TrackerPos[2]) // 2 - 320)/320
                xc = xc * 90
                
                angle = int(xc)
                if depth_value < threshold_depth:
                    speed = 0
                else:
                    speed = 5
                self.get_logger().info(f"{speed}, {angle}")

                publicData = [speed, angle]
                msg = Float32MultiArray()
                msg.data = publicData
                self.publisher_.publish(msg)

        # cv2.imshow("frame", out_show)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     self.finish_detect()
        #     break
        


def main(arg=None):
    rclpy.init()
    node = PostProcessing()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
