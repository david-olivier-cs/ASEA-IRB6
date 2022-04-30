'''
Controls the ASEA IRB6 manipulator by specifying the axis angles

Usage
-----
python3 asea_control_angles.py (config path)
'''

import time
import argparse

from init_irb6.controller import ASEAController


if __name__ == "__main__" : 
	
	# parsing script arguments
	parser = argparse.ArgumentParser()
	parser.add_argument("config_path", help="path to the .json config file")
	args = parser.parse_args()

	# robotic arm controller
	asea_controller = ASEAController(args.config_path, selected_axes=[1, 2, 3, 4, 5])

	# defining the position selection vars
	position_i = 0
	target_positions = [(-172.5, 130, 10, 90, 180), (0, 90, 0, 0, 0), (172.5 , 50, -40, -90, -180), (0, 90, 0, 0, 0)]

	# homing the robotic arm
	print("homing - start")
	asea_controller.home()
	print("homing - end")

	# main control loop
	while True:

		print(f"target positions : {target_positions[position_i]}")

		# assigning positions to the axes controlled by (velocity) controllers
		for axis_i, joint_pos in enumerate(target_positions[position_i]):
			if (joint_pos is not None) and (asea_controller.axes[axis_i]):
				if asea_controller.axes[axis_i].axis_type == "velocity":
					asea_controller.set_axis_position(axis_i + 1, joint_pos)
		
		# assigning positions to the axes controlled by (position) controllers
		for axis_i, joint_pos in enumerate(target_positions[position_i]):
			if (joint_pos is not None) and (asea_controller.axes[axis_i]):
				if asea_controller.axes[axis_i].axis_type == "position":
					asea_controller.set_axis_position(axis_i + 1, joint_pos)

		time.sleep(10)

		# defining the next movement
		position_i = (position_i + 1) % len(target_positions)