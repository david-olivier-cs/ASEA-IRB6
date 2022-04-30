'''
Moves the ASEA IRB6 manipulator to a series of predefined points upon bystander movement detection

Usage
-----
python3 main.py (config path)
'''

import time
import math
import argparse
import threading

from init_irb6.controller import ASEAController
from init_irb6.simulation import ASEAMovementGenerator
from init_irb6.gpio import MovementSensor, LightIndicator


if __name__ == "__main__" : 

    # parsing script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("config_path", help="path to the .json config file")
    args = parser.parse_args()

    # creating the robot simulation and controller
    movement_generator = ASEAMovementGenerator(args.config_path)
    movement_generator.simulation.joint_ranges[0] = (0, 0.25 * math.pi)
    asea_controller = ASEAController(args.config_path, selected_axes=[1, 2, 3, 4, 5])

    # creating the movement sensor + light indicator
    light_indicator = LightIndicator(17, 27)
    movement_sensor = MovementSensor(args.config_path)

    # initial homing of the robot
    light_indicator.set_red_light_state(True)
    light_indicator.set_green_light_state(False)
    asea_controller.home()
    time.sleep(5)
    light_indicator.set_red_light_state(False)
    light_indicator.set_green_light_state(True)
    time.sleep(2)
    
    # defining the number of movements between each homing sequence
    mov_i = 0
    n_movements = 10
    
    # main control loop
    while True:

        # initiating movement when bystander movement is detected
        if movement_sensor.check_movement():

            # turning on the red light
            light_indicator.set_red_light_state(True)
            light_indicator.set_green_light_state(False)

            # generating random angles
            _, target_joint_angles = next(movement_generator)
            
            # moving the waist axis first (looks cooler)
            for axis_i, joint_pos in enumerate(target_joint_angles):
                if (joint_pos is not None) and (asea_controller.axes[axis_i]):
                    if asea_controller.axes[axis_i].ax_id == "ax1":
                        asea_controller.set_axis_position(axis_i + 1, joint_pos)
            
            # moving all the the other axes
            control_threads = []
            for axis_i, joint_pos in enumerate(target_joint_angles):
                if (joint_pos is not None) and (asea_controller.axes[axis_i]):
                    if not asea_controller.axes[axis_i].ax_id == "ax1":
                        control_threads.append(threading.Thread(target=asea_controller.axes[axis_i].set_position, args=(joint_pos,)))
                        control_threads[-1].start()
            for t in control_threads: t.join()
            
            # turning on the red light
            light_indicator.set_red_light_state(False)
            light_indicator.set_green_light_state(True)
            
            # defining the next movement
            # homing the robot before going back to first target point + reset movement sensor
            mov_i = (mov_i + 1) % n_movements
            if mov_i == 0: 
                
                time.sleep(30)
                
                light_indicator.set_red_light_state(True)
                light_indicator.set_green_light_state(False)
                asea_controller.home()
                light_indicator.set_red_light_state(False)
                light_indicator.set_green_light_state(True)
                
                movement_sensor.reset()

        time.sleep(0.5)