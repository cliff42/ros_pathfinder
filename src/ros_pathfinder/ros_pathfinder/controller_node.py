#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gpiozero import PhaseEnableMotor

class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.flag = True
        #TODO
        pass
    
def main(args=None):
    #TODO
    pass

if __name__ == "__main__":
    main()