import struct

import can
from std_msgs.msg import Float32MultiArray

import rclpy
from rclpy.node import Node


class get_control(Node):
    def __init__(self):
        super().__init__('get_control')
        self.bus = can.Bus(interface='socketcan', channel='can0')
        self.speed = 0
        self.angle = 90
        self.set_value()
        self.subcriber_ = self.create_subscription(Float32MultiArray, 
                                                   'get_control', 
                                                   self.control, 
                                                   1)
    
    def set_value(self):
        data = struct.pack('>hh', self.speed, 90 + self.angle)
        msg = can.Message(
            arbitration_id=0x21, data=data, is_extended_id=False
        )
        try:
            self.bus.send(msg)
            # print(f"Message sent on {self.bus.channel_info}")
        except can.CanError:
            print("Message NOT sent")

    def control(self, msg):
        speed, angle = msg.data
        if speed == self.speed and angle == self.angle:
            return
        # self.speed = msg.speed
        # self.angle = msg.angle
        if self.speed >= 100:
            self.speed = 100
        if self.speed <= -100:
            self.speed = -100
        if self.angle >= 90:
            self.angle = 90
        if self.angle <= -90:
            self.angle = -90
        self.set_value()

def main(arg=None):
    rclpy.init()
    node = get_control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()