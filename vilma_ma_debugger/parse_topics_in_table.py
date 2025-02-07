
#   ******************************************************************************
#   * @file    parse_topics_in_table.py
#   * @brief   This file contains the functions and constants, to extract and
#   *          parse the topics data in a table.
#   ******************************************************************************
#   * @author  Gabriel Toffanetto França da Rocha 
#   *          Laboratory of Autonomous Vehicles (LMA) - FEM/Unicamp
#   * @date    Created:  February 06, 2025
#   *          Modified: February 06, 2025
#   ******************************************************************************

from rich.console import Console
from rich.table import Table
from rich.text import Text
from rich import box
from itertools import zip_longest


# Labeling

sensors_ma_labels = ['ROS Time', 'Time PCC', 'Operation state', 'Steer position', 'Steer speed', 'Steer state', 'Steer Vm', 'Steer user torque', 'Gas value', 
                     'Gas user value', 'Current sensor', 'Acc counter', 'Acc x', 'Acc y', 'Acc z', 'Pressure', 'Time PCC Brake', 
                     'Brake state', 'Brake value', 'Brake ws front left', 'Brake ws front right', 'Brake ws rear left', 'Brake ws rear right',
                     'Brake Acc x', 'Brake Acc y', 'Brake Acc z', 'Brake Gyr z', 'Brake user pressure', 'Brake front encoder', '', 'Gear state']

sensors_ma_gear_states = ['Off', 'N', 'R', 'D', '-', '+', 'A/M']

sensors_ma_units = ['s', 's', '', 'rad', 'rad/s', '', 'V', 'Nm', '%', '%', 'A', '', 'g', 'g', 'g', 'kPa', 
                    's', '', '%', 'km/h', 'km/h', 'km/h', 'km/h', 'g', 'g', 'g', 'degree/s', 'bar', 'pulses/s', '', '']


state_ma_labels = ['ROS Time', 'Time PPC', 'Steer angle', 'Steer speed angle', 'Tire angle', 'Tire speed angle', 'Lateral velocity', 'Angular yaw speed', 
                  'X global position', 'Y global position', 'Yaw angle', 'Longitudinal speed', 'User torque', 'Steer motor voltage', 'Yaw DC error']

state_ma_units = ['s', 's', 'rad', 'rad/s', 'rad', 'rad/s', 'm/s', 'rad/s', 'm', 'm', 'rad', 'm/s', 'Nm', 'V', 'rad']

joystick_ma_labels = ['ROS Time', 'Time validity', 'Brake command', 'Brake value', 'Steer command', 'Steer value', 'Gas command', 'Gas value', 'Gear state', 'Gear value']

joystick_ma_units = ['s', 'ms', '', '', '', '', '', '', '', '']

joystick_ma_brake_modes = ['Manual', '', 'Engaged']
joystick_ma_steer_modes = ['Manual', 'Angular velocity', 'Steer angle', 'Motor voltage', 'Find zero position']
joystick_ma_gas_modes = ['Manual', 'Gas pedal velocity', 'Gas pedal position']
joystick_ma_gear_modes = ['Manual', 'N', 'R', 'D']



# States code
EMERGENCY_STATE         = 1
INITIAL_STATE           = 2
MANUAL_STATE            = 4
JOYSTICK_STATE          = 8
AUTOMATIC_STATE         = 16
ADAS_STATE              = 32
EMERGENCY_BUTTON        = 256
STATE_ERROR             = 512
RX_UDP_ERROR            = 1024
TX_UDP_ERROR            = 2048
ACCELERATOR_USER_ERROR  = 4096
TX_RS232_ERROR          = 8192
STEER_ERROR             = 16384
BRAKE_ERROR             = 32768
STATE_ESTIMATION_ERROR  = 65536
CONTROLLER_ERROR        = 131072

STEER_TURN_OFF_MOTOR            = 0
STEER_VELOCITY_MODE             = 1
STEER_POSITION_MODE             = 2
STEER_TORQUE_MODE               = 3
STEER_HOME_MODE                 = 4
STEER_SERVO_BUS_VOLTAGE_ERROR   = 8
STEER_PEAK_OVER_CURRENT         = 16
STEER_EXCESSIVE_TEMPERATURE     = 32
STEER_EXCESSIVE_ERROR_POSITION  = 64
STEER_COM_ERROR                 = 128
STEER_PWM_SATURATION            = 256
STEER_RIGHT_OVER_TRAVEL         = 512
STEER_LEFT_OVER_TRAVEL          = 1024

operation_state_codes = [EMERGENCY_STATE, INITIAL_STATE, MANUAL_STATE, JOYSTICK_STATE, AUTOMATIC_STATE, ADAS_STATE, 
                 EMERGENCY_BUTTON, STATE_ERROR, RX_UDP_ERROR, TX_UDP_ERROR, ACCELERATOR_USER_ERROR, STEER_ERROR,
                 BRAKE_ERROR, STATE_ESTIMATION_ERROR, CONTROLLER_ERROR]

operation_state_labels = ['EMERGENCY_STATE', 'INITIAL_STATE', 'MANUAL_STATE', 'JOYSTICK_STATE', 'AUTOMATIC_STATE', 'ADAS_STATE', 
                 'EMERGENCY_BUTTON', 'STATE_ERROR', 'RX_UDP_ERROR', 'TX_UDP_ERROR', 'ACCELERATOR_USER_ERROR', 'STEER_ERROR',
                 'BRAKE_ERROR', 'STATE_ESTIMATION_ERROR', 'CONTROLLER_ERROR']

steer_state_codes = [STEER_TURN_OFF_MOTOR, STEER_VELOCITY_MODE, STEER_POSITION_MODE, STEER_TORQUE_MODE, STEER_HOME_MODE, STEER_SERVO_BUS_VOLTAGE_ERROR,
                     STEER_PEAK_OVER_CURRENT, STEER_EXCESSIVE_TEMPERATURE, STEER_EXCESSIVE_ERROR_POSITION, STEER_COM_ERROR,
                     STEER_PWM_SATURATION, STEER_RIGHT_OVER_TRAVEL, STEER_LEFT_OVER_TRAVEL]

steer_state_labels = ['STEER_TURN_OFF_MOTOR', 'STEER_VELOCITY_MODE', 'STEER_POSITION_MODE', 'STEER_TORQUE_MODE', 'STEER_HOME_MODE', 'STEER_SERVO_BUS_VOLTAGE_ERROR',
                     'STEER_PEAK_OVER_CURRENT', 'STEER_EXCESSIVE_TEMPERATURE', 'STEER_EXCESSIVE_ERROR_POSITION', 'STEER_COM_ERROR',
                     'STEER_PWM_SATURATION', 'STEER_RIGHT_OVER_TRAVEL', 'STEER_LEFT_OVER_TRAVEL']



def decode_state_data(code, list_of_codes):
    if code == 0:
        return [1 if (code & state == state) else 0 for state in list_of_codes]
    else:
        return [1 if (code & state == state and code & state != 0) else 0 for state in list_of_codes]
    
    
    


def get_steer_rows(steer_code):
    # Steering states    
        
    steer_rows = []

    steer_states = decode_state_data(int(steer_code),steer_state_codes)

    i = 0
    while i < len(steer_states):
        if 1 == i:
            if steer_states[3]:
                steer_rows.append([Text(steer_state_labels[3],style='bright_green')])
            elif steer_states[2]:
                steer_rows.append([Text(steer_state_labels[2],style='bright_green')])
            elif steer_states[1]:
                steer_rows.append([Text(steer_state_labels[1],style='bright_green')])
            i = 4
        
        if steer_states[i] == 1:
            steer_rows.append([Text(steer_state_labels[i],style='bright_green')])
            
        i = i + 1
        
    return steer_rows

def get_operation_rows(operation_code):
        
    # Operation states
        
    operation_rows = []

    operation_states = decode_state_data(int(operation_code),operation_state_codes)

    i = 0
    while i < len(operation_states):
        
        if operation_states[i] == 1:
            operation_rows.append([Text(operation_state_labels[i],style='bright_green')])
            
        i = i + 1    
        
    return operation_rows

def get_joystick_rows(joy):
        
    # Joystick_ma topic data

    joystick_ma_rows = []

    i = 0
    while i < len(joy):
        
        if(2 == i):
            try:
                joy_value = joystick_ma_brake_modes[int(joy[i])]
            except:
                joy_value = '--'
            
        elif(4 == i):
            try:
                joy_value = joystick_ma_steer_modes[int(joy[i])]
            except:
                joy_value = '--'
            
        elif(6 == i):
            try:
                joy_value = joystick_ma_gas_modes[int(joy[i])]
            except:
                joy_value = '--'
            
        elif(8 == i):
            try:
                joy_value = joystick_ma_gear_modes[int(joy[i])]
            except:
                joy_value = '--'
            
        else:
            joy_value = joy[i]
        
        joystick_ma_rows.append([Text(joystick_ma_labels[i],style='bright_green'), Text(str(joy_value)+' '+joystick_ma_units[i],style='bright_green')])
            
        i = i + 1
        
    return joystick_ma_rows

def get_sensors_rows(sensors):
    # Sensors_ma topic data

    sensors_ma_rows = []

    i = 0
    while i < len(sensors):
        
        if(2 == i or 5 == i or 29 == i):
            i = i + 1
            continue
        
        if(i == 30):
            try:
                sensors_ma_rows.append([Text(sensors_ma_labels[i],style='bright_green'), Text(str(sensors_ma_gear_states[sensors[i]]), style='bright_green')])
            except:
                sensors_ma_rows.append([Text(sensors_ma_labels[i],style='bright_green'), Text('--', style='bright_green')])
        else:
            sensors_ma_rows.append([Text(sensors_ma_labels[i],style='bright_green'), Text(str(sensors[i])+' '+sensors_ma_units[i], style='bright_green')])

        i = i + 1
        
    return sensors_ma_rows

def get_state_rows(state):
        
    # State_ma topic data

    state_ma_rows = []

    i = 0
    while i < len(state):
        
        state_ma_rows.append([Text(state_ma_labels[i],style='bright_green'), Text(str(state[i])+' '+state_ma_units[i], style='bright_green')])

        i = i + 1
        
    return state_ma_rows


def parse_and_print_table(sensors_ma_rows, state_ma_rows, joystick_ma_rows, steer_rows, operation_rows):
        
    joystick_ma_columns = [Text('joystick_ma',style='bold bright_white'), Text('Value',style='bold bright_white')]

    sensors_ma_columns = ['sensors_ma', 'Value']
        
    state_ma_columns = ['state_ma', 'Value']

    # Parsing data
            
    table_vilma_status = Table(title='VILMA MA status', box=box.SQUARE)

    vilma_columns = [*sensors_ma_columns, *state_ma_columns, 'Vehicle state']
        
    for column in vilma_columns:
        table_vilma_status.add_column(column, justify='center', vertical='middle', no_wrap=True)

    ## Vehicle state data
        
    vehicle_state_rows = [[Text('Operation state', style='bold bright_white')]]+operation_rows+[['─────────────────────────────']]+[[Text('Steer state', style='bold bright_white')]]+steer_rows

    ## VILMA ma data

    state_and_joystick_ma_rows = state_ma_rows+[['───────────────────','───────────────────']]+[joystick_ma_columns]+joystick_ma_rows

    for sensors_ma_entry, state_and_joystick_ma_entry, vehicle_state_entry in zip_longest(sensors_ma_rows, state_and_joystick_ma_rows, vehicle_state_rows, fillvalue=' '):
        table_vilma_status.add_row(*sensors_ma_entry, *state_and_joystick_ma_entry, *vehicle_state_entry)
        
    console = Console()
    # console.clear()
    console.print(table_vilma_status)  

    # Clearing new data colors
    for sensors_ma_entry, state_ma_entry, joystick_ma_entry, operation_state, steer_state  in zip_longest(sensors_ma_rows, state_ma_rows, joystick_ma_rows, operation_rows, steer_rows, fillvalue=' '):
        try:
            sensors_ma_entry[0].stylize('bright_white')
            sensors_ma_entry[1].stylize('bright_white')
        except:
            None

        try:
            state_ma_entry[0].stylize('bright_white')
            state_ma_entry[1].stylize('bright_white')
        except:
            None

        try:
            joystick_ma_entry[0].stylize('bright_white')
            joystick_ma_entry[1].stylize('bright_white')
        except:
            None  
        try:
            operation_state[0].stylize('bright_white')
        except:
            None

        try:
            steer_state[0].stylize('bright_white')
        except:
            None    
    

def main():
    
    ### Test variables

    import random

    sensors = [random.randint(0, 10) for _ in range(31)]

    state = [random.randint(0, 100) for _ in range(15)]

    joy = [134134, 34234, 2, 0.5, 2, -0.5, 2, 0.7, 2, 0]

    steer_code = 2 + 128 + 32 + 64
    operation_code = 2 + 128 + 32 + 64
    
    sensors_ma_rows = get_sensors_rows(sensors=sensors)
    
    operation_rows = get_operation_rows(operation_code=sensors[2])
    
    steer_rows = get_steer_rows(steer_code=sensors[5])
    
    state_ma_rows = get_state_rows(state=state)
        
    joystick_ma_rows = get_joystick_rows(joy=joy)
    
    parse_and_print_table(sensors_ma_rows, state_ma_rows, joystick_ma_rows, steer_rows, operation_rows)
    
if __name__ == '__main__':
    main()
