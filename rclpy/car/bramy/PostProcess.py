import time

import cv2
import numpy as np
import onnxruntime as ort
import torch
from bl_msg.msg import Control
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import rclpy
from bramy.ocsort.ocsort import OCSort
from rclpy.node import Node

#640 640
trackerID = None
TrackerPos = None

def plot_bboxes(image, bboxes, line_thickness=None):
    global trackerID, TrackerPos
    # Plots one bounding box on image img
    tl = 1


    selecting = False
    if not trackerID:
        selecting = True
        
        centerX = image.shape[1] // 2
        centerY = image.shape[0] // 2
        maxDist = float('inf')
    cls_id = 'person'
    for (x1, y1, x2, y2, pos_id) in bboxes:
        if int(pos_id) == trackerID:
            TrackerPos = (x1 + x2) // 2, (y1+y2) //2
            color = (255, 0, 255)
            # print(color)
        else:
            color = (0, 255, 0)

        c1, c2 = (x1, y1), (x2, y2)
        cv2.rectangle(image, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
        if selecting:
            x_center = (x1 + x2) // 2
            y_center = (y1 + y2) // 2
            dist = ((centerX - x_center) ** 2 + (centerY - y_center) ** 2)**0.5
            print(pos_id, dist)
            if dist < maxDist:
                trackerID = int(pos_id)
                maxDist = dist

        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(cls_id, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(image, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(image, '{} ID-{}'.format(cls_id, pos_id), (c1[0], c1[1] - 2), 0, tl / 3,
                    [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)


    return image

ocSort = OCSort()

        
class PostProcessing(Node):
    def __init__(self):
        super().__init__('post_prcesss')

        self.create_subscription(Image, 'cordinates', self.detect, 1)
        self.publisher_ = self.create_publisher(Control, 'get_control', 1)
        self.bridge = CvBridge()
        self.trackerID = None
        self.TrackerPos = None
    
    def getCenterBox(self, bboxes):
        tl = 1
        selecting = False
        if not trackerID:
            selecting = True
            centerX = 640 // 2
            centerY = 480 // 2
            maxDist = float('inf')


        for (x1, y1, x2, y2, pos_id) in bboxes:
            if int(pos_id) == trackerID:
                self.TrackerPos = (x1 + x2) // 2, (y1+y2) //2
                color = (255, 0, 255)
                # print(color)
            else:
                color = (0, 255, 0)

            c1, c2 = (x1, y1), (x2, y2)
            # cv2.rectangle(image, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
            if selecting:
                x_center = (x1 + x2) // 2
                y_center = (y1 + y2) // 2
                dist = ((centerX - x_center) ** 2 + (centerY - y_center) ** 2)**0.5
                print(pos_id, dist)
                if dist < maxDist:
                    self.trackerID = int(pos_id)
                    self.TrackerPos = (x1 + x2) // 2, (y1+y2) //2
                    maxDist = dist


            # cv2.putText(image, '{} ID-{}'.format(cls_id, pos_id), (c1[0], c1[1] - 2), 0, 1,
            #             [225, 255, 255], thickness=1, lineType=cv2.LINE_AA)

    def detect(self, cordinates):
        prev_time = time.time()
        bboxes = cordinates.data
        xywhs = torch.tensor(bboxes)
        if not bboxes:
            print("No object!")
            trackerID = None
            TrackerPos = None
        else:
            if len(bboxes) == 1:
                if bboxes[0][-2] < 0.4:
                    return
                else:
                    x1, y1, x2, y2, cls_id, depth_value = bboxes[0]
                    TrackerPos = (x1 + x2) // 2, (y1+y2) //2
            else:
                outputs = ocSort.update(xywhs, (640,480), (640,480)).astype(np.int32)
                self.getCenterBox(outputs)
            

            if TrackerPos != None or (multi and TrackerPos):
                threshold_depth = 1000
                person_x, person_y = TrackerPos
                xc = (person_x - 320)/320
                xc = xc * 90


                if depth_value < threshold_depth:
                    msg.speed = 0
                else:
                    msg.speed = 5
                    msg.angle = int(xc)
                    self.angle = int(xc)
                    self.publisher_.publish(msg)
                else:
                    msg.speed = 0
                    msg.angle = self.angle
                    self.publisher_.publish(msg)

        current_time = time.time()
        fps = 1.0 / (current_time - prev_time)
        prev_time = current_time
        print(fps)
        cv2.imshow("frame", out_show)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.finish_detect()
            break
        


def main(arg=None):
    rclpy.init()
    node = PosProcessing()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
