#!/usr/bin/env python3

# Copyright 2021 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Wonho Yun, Will Son


from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk import *
from std_msgs.msg import Int32MultiArray
from crab_interfaces.msg import ServoData

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# Control table address for 
ADDR_OPERATING_MODE = 11 # Control table address is different in Dynamixel model
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_HOMING_OFFSET = 20

# Protocol version
PROTOCOL_VERSION = 2.0  # Default Protocol version of DYNAMIXEL X series.

# Default settings
BAUDRATE = 1000000  # Dynamixel default baudrate : 57600
DEVICE_NAME = '/dev/ttyUSB0'  # Check which port is being used on your controller

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
# 4096 pulses per revolution
POSITION_CONTROL = 3  # Value 4 for Extended position control mode, 3 for normal position control
DATA_LENGTH_4BYTE = 4

# Amount of servos to init on the bus (MAX 4)
<<<<<<< HEAD
NUM_SERVOS = 4
HOMING_OFFSET = 410
=======
NUM_SERVOS = 2
HOMING_OFFSET = -1000
>>>>>>> 2ef349f (rebase and merge off main. built but not run)


class ReadWriteNode(Node):
    def __init__(self):
        super().__init__('read_write_node')

        # List of servo goals and positions
        self.servo_goals = [0,0,0,0]
        self.servo_positions = [0,0,0,0]

        
        self.port_handler = PortHandler(DEVICE_NAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error('Failed to open the port!')
            return
        self.get_logger().info('Succeeded to open the port.')

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error('Failed to set the baudrate!')
            return
        self.get_logger().info('Succeeded to set the baudrate.')

        # Create group sync read and write objects
        self.groupSyncWrite = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_GOAL_POSITION, DATA_LENGTH_4BYTE)
        self.groupSyncRead = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_PRESENT_POSITION, DATA_LENGTH_4BYTE)

        # Setup N Servos (4 Max for this project)
        for i in range(1, NUM_SERVOS+1):
            self.setup_dynamixel(i)
        
        for dxl_id in range(1, NUM_SERVOS + 1):
            if not self.groupSyncRead.addParam(dxl_id):
                self.get_logger().error(f"Failed to add servo {dxl_id} to GroupSyncRead")

        # Setup the subscriber for servo goals along with the service to get feedback
        qos = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            SetPosition,
            f'servo/set_position',
            self.set_position_callback,
            qos
        )
        self.timer = self.create_timer(0.05, self.get_position_callback)
        self.pospub = self.create_publisher(ServoData, '/servo/position_data', qos)
        self.encpub = self.create_publisher(ServoData, '/servo/encoder_data', qos)
        

    def setup_dynamixel(self, dxl_id):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_OPERATING_MODE, POSITION_CONTROL
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set Position Control Mode: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('Succeeded to set Position Control Mode.')


        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, ADDR_HOMING_OFFSET, HOMING_OFFSET
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set Homing Offset: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('Succeeded to set Homing Offset.')


        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to enable torque: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('Succeeded to enable torque.')

    def group_sync_write(self, goals):
        # Package goals
        for idx, goal in enumerate(goals):
                param_goal_position = [
                    DXL_LOBYTE(DXL_LOWORD(goal)),
                    DXL_HIBYTE(DXL_LOWORD(goal)),
                    DXL_LOBYTE(DXL_HIWORD(goal)),
                    DXL_HIBYTE(DXL_HIWORD(goal))
                ]
                dxl_addparam_result = self.groupSyncWrite.addParam(idx+1, param_goal_position)
                if not dxl_addparam_result:
                    self.get_logger().error("[ID:%03d] groupSyncWrite addparam failed" % (idx+1))
        # Send Goals
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        self.groupSyncWrite.clearParam()
    
    def set_position_callback(self, msg):
        # Update goal array
        self.servo_goals[msg.id-1] = msg.position
        # Send out the array
        self.group_sync_write(self.servo_goals)
        
        # Publish current servo commands
        log = ServoData()
        log.header.stamp = self.get_clock().now().to_msg()
        log.data = self.servo_goals
        self.pospub.publish(log)
    

    def get_position_callback(self):
        
        msg = ServoData()
        
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        
        for idx, position in enumerate(self.servo_positions):
            self.servo_positions[idx] = self.groupSyncRead.getData(idx+1, ADDR_PRESENT_POSITION, DATA_LENGTH_4BYTE)
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = self.servo_positions
        self.encpub.publish(msg)


    def __del__(self):
        self.packet_handler.write1ByteTxRx(self.port_handler,
                                           1,
                                           ADDR_TORQUE_ENABLE,
                                           TORQUE_DISABLE)
        self.port_handler.closePort()
        self.get_logger().info('Shutting down read_write_node')


def main(args=None):
    rclpy.init(args=args)
    node = ReadWriteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
