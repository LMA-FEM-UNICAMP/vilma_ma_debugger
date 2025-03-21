
#   ******************************************************************************
#   * @file    vilma_ma_status.py
#   * @brief   This file contains the vilma_ma_status node class, that read the
#   *          vilma_ma_ros topics and parses it in a table.
#   ******************************************************************************
#   * @author  Gabriel Toffanetto França da Rocha 
#   *          Laboratory of Autonomous Vehicles (LMA) - FEM/Unicamp
#   * @date    Created:  February 06, 2025
#   *          Modified: February 06, 2025
#   ******************************************************************************

 


import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from vilma_ma_debugger.parse_topics_in_table import parse_and_print_table, get_joystick_rows, get_operation_rows, get_sensors_rows, get_state_rows, get_steer_rows


class VilmaMaStatus(Node):
    
    def __init__(self):
        super().__init__('vilma_ma_status')
        
        self.declare_parameter('update_period', 0.3)
        
        timer_period = self.get_parameter('update_period').value
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.sensors_ma_sub_ = self.create_subscription(
            Float64MultiArray,
            '/vilma_ma_debug/sensors_ma',
            self.sensors_ma_callback,
            1)
        self.sensors_ma_sub_
        
        self.state_ma_sub_ = self.create_subscription(
            Float64MultiArray,
            '/vilma_ma_debug/state_ma',
            self.state_ma_callback,
            1)
        self.state_ma_sub_
        
        self.joystick_ma_sub_ = self.create_subscription(
            Float64MultiArray,
            '/vilma_ma_debug/joystick_ma',
            self.joystick_ma_callback,
            1)
        self.joystick_ma_sub_
        
        
        self.sensors_ma_rows = []
        self.state_ma_rows = []
        self.joystick_ma_rows = []
        self.operation_rows = []
        self.steer_rows = []
        
    def timer_callback(self):
        
        parse_and_print_table(sensors_ma_rows=self.sensors_ma_rows,
                              state_ma_rows=self.state_ma_rows,
                              joystick_ma_rows=self.joystick_ma_rows,
                              operation_rows=self.operation_rows,
                              steer_rows=self.steer_rows)
        
    def sensors_ma_callback(self, sensors_ma_msg):
        
        self.sensors_ma_rows = get_sensors_rows(sensors=sensors_ma_msg.data)
        self.operation_rows = get_operation_rows(operation_code=sensors_ma_msg.data[2])
        self.steer_rows = get_steer_rows(steer_code=sensors_ma_msg.data[5])
        
    def state_ma_callback(self, state_ma_msg):
        
        self.state_ma_rows = get_state_rows(state=state_ma_msg.data)
        
    def joystick_ma_callback(self, joystick_ma_msg):
        
        self.joystick_ma_rows = get_joystick_rows(joy=joystick_ma_msg.data)



def main(args=None):
    rclpy.init(args=args)
    
    vilma_ma_status_node = VilmaMaStatus()
    
    rclpy.spin(vilma_ma_status_node)
    
    vilma_ma_status_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
