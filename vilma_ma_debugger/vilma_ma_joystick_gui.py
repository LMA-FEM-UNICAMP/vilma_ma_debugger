
#   ******************************************************************************
#   * @file    vilma_ma_joystick_gui_node.py
#   * @brief   
#   ******************************************************************************
#   * @author  Gabriel Toffanetto França da Rocha 
#   *          Laboratory of Autonomous Vehicles (LMA) - FEM/Unicamp
#   * @date    Created:  February 06, 2025
#   *          Modified: 
#   ******************************************************************************

 


import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


import tkinter as tk
from tkinter import ttk


class VilmaMaJoystickGui(Node):
    
    def __init__(self):
        super().__init__('vilma_ma_joystick_gui')
        
        self.declare_parameter('ma_communication', 'udp')
        
        ma_communication = self.get_parameter('ma_communication').value
        
        if ('udp' == ma_communication):
            joystick_ma_topic = '/vilma_ma_debug/joystick_ma'
        elif ('bridge' == ma_communication):
            joystick_ma_topic = '/vilma_ma_ros/joystick_ma'
        
        self.joystick_ma_pub_ = self.create_publisher(Float64MultiArray, joystick_ma_topic, 1)

        self.joystick_msg = Float64MultiArray()
        self.joystick_msg.data = [0.0]*10
        
        self.joystick_msg.data[1] = 60*5*1000
        
        
        self.steer_modes = ["Off", "Velocity", "Position", "Voltage", "Find zero"]
        self.gas_modes   = ["Off", "Pedal speed", "Pedal position"]
        self.brake_modes = ["Off", "Percentage"]
        self.gear_modes  = ["Off", "N", "R", "D"]
        self.gear_value  = ["-", "+", "Auto/Manual"]


        # Create GUI

        self.root = tk.Tk()
        self.root.title("VILMA MA Joystick GUI")
        self.root.geometry("600x330")

        # Create sliders

        self.steer_slider = tk.Scale(self.root, from_=1, to=-1, resolution=0.01, orient='horizontal', label="Steering")
        self.steer_slider.pack(fill='x')
        self.steer_slider.set(0)
        self.steer_slider.config(command=self.slider_update)

        self.gas_slider = tk.Scale(self.root, from_=0, to=1, resolution=0.01, orient='horizontal', label="Gas pedal")
        self.gas_slider.pack(fill='x')
        self.gas_slider.set(0)
        self.gas_slider.config(command=self.slider_update)

        self.brake_slider = tk.Scale(self.root, from_=0, to=1, resolution=0.01, orient='horizontal', label="Brake pedal")
        self.brake_slider.pack(fill='x')
        self.brake_slider.set(0)
        self.brake_slider.config(command=self.slider_update)


        # Create _dropdown lists

        self.frame = tk.Frame(self.root)
        self.frame.pack()


        self.steer_modes_label = tk.Label(self.frame, text=f"Steer mode: ")
        self.steer_modes_label.grid(row=0, column=2)
        self.steer_modes_var = tk.StringVar()
        self.steer_modes_var.set(self.steer_modes[0])
        self.steer_modes_dropdown = ttk.OptionMenu(self.frame, self.steer_modes_var, self.steer_modes[0], *self.steer_modes, 
                                              command=lambda value, v=self.steer_modes_var, name=f"steer_modes": self.on_dropdown_change(v, name))
        self.steer_modes_dropdown.grid(row=0, column=3)


        self.gas_modes_label = tk.Label(self.frame, text=f"Gas mode: ")
        self.gas_modes_label.grid(row=1, column=2)
        self.gas_modes_var = tk.StringVar()
        self.gas_modes_var.set(self.gas_modes[0])
        self.gas_modes_dropdown = ttk.OptionMenu(self.frame, self.gas_modes_var, self.gas_modes[0], *self.gas_modes, 
                                            command=lambda value, v=self.gas_modes_var, name=f"gas_modes": self.on_dropdown_change(v, name))
        self.gas_modes_dropdown.grid(row=1, column=3)


        self.brake_modes_label = tk.Label(self.frame, text=f"Brake mode: ")
        self.brake_modes_label.grid(row=2, column=2)
        self.brake_modes_var = tk.StringVar()
        self.brake_modes_var.set(self.brake_modes[0])
        self.brake_modes_dropdown = ttk.OptionMenu(self.frame, self.brake_modes_var, self.brake_modes[0], *self.brake_modes, 
                                              command=lambda value, v=self.brake_modes_var, name=f"brake_modes": self.on_dropdown_change(v, name))
        self.brake_modes_dropdown.grid(row=2, column=3)


        self.gear_modes_label = tk.Label(self.frame, text=f"Gear mode: ")
        self.gear_modes_label.grid(row=3, column=2)
        self.gear_modes_var = tk.StringVar()
        self.gear_modes_var.set(self.gear_modes[0])
        self.gear_modes_dropdown = ttk.OptionMenu(self.frame, self.gear_modes_var, self.gear_modes[0], *self.gear_modes, 
                                             command=lambda value, v=self.gear_modes_var, name=f"gear_modes": self.on_dropdown_change(v, name))
        self.gear_modes_dropdown.grid(row=3, column=3)


        self.gear_value_label = tk.Label(self.frame, text=f"Gear value: ")
        self.gear_value_label.grid(row=4, column=2)
        self.gear_value_var = tk.StringVar()
        self.gear_value_var.set(self.gear_value[0])
        self.gear_value_dropdown = ttk.OptionMenu(self.frame, self.gear_value_var, self.gear_value[0], *self.gear_value, 
                                             command=lambda value, v=self.gear_value_var, name=f"gear_value": self.on_dropdown_change(v, name))
        self.gear_value_dropdown.grid(row=4, column=3)


        self.root.mainloop()
        
        
        
    def slider_update(self,*args):
        
        self.joystick_msg.data[3] = self.brake_slider.get()
        self.joystick_msg.data[5] = self.steer_slider.get()
        self.joystick_msg.data[7] = self.gas_slider.get()
        
        self.joystick_msg.data[0] = float(self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec*1.0e-9)
        self.joystick_ma_pub_.publish(self.joystick_msg)
        

    # Callback function for _dropdown selections

    def on_dropdown_change(self, var, dropdown_name):
        
        if(dropdown_name == "steer_modes"):
            if(var.get() == self.steer_modes[0]):
                value = 0
                
            elif(var.get() == self.steer_modes[1]):
                value = 1
                
            elif(var.get() == self.steer_modes[2]):
                value = 2
                
            elif(var.get() == self.steer_modes[3]):
                value = 3
                
            elif(var.get() == self.steer_modes[4]):
                value = 4
            
            self.joystick_msg.data[4] = value
                
        elif(dropdown_name == "gas_modes"):
            if(var.get() == self.gas_modes[0]):
                value = 0
                
            elif(var.get() == self.gas_modes[1]):
                value = 1
                
            elif(var.get() == self.gas_modes[2]):
                value = 2
            
            self.joystick_msg.data[6] = value
                            
        elif(dropdown_name == "brake_modes"):
            if(var.get() == self.brake_modes[0]):
                value = 0
                
            elif(var.get() == self.brake_modes[1]):
                value = 2
            
            self.joystick_msg.data[2] = value
                
        elif(dropdown_name == "gear_modes"):
            if(var.get() == self.gear_modes[0]):
                value = 0
                
            elif(var.get() == self.gear_modes[1]):
                value = 1
                
            elif(var.get() == self.gear_modes[2]):
                value = 2
                
            elif(var.get() == self.gear_modes[3]):
                value = 3
            
            self.joystick_msg.data[8] = value
                
        elif(dropdown_name == "gear_value"):
            if(var.get() == self.gear_value[0]):
                value = 0
                
            elif(var.get() == self.gear_value[1]):
                value = 1
                
            elif(var.get() == self.gear_value[2]):
                value = 3
            
            self.joystick_msg.data[9] = value
        
        self.joystick_msg.data[0] = float(self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec*1.0e-9)
        self.joystick_ma_pub_.publish(self.joystick_msg)
                
        



def main(args=None):
    rclpy.init(args=args)
    
    vilma_ma_joystick_gui_node = VilmaMaJoystickGui()
    
    rclpy.spin(vilma_ma_joystick_gui_node)
    
    vilma_ma_joystick_gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
