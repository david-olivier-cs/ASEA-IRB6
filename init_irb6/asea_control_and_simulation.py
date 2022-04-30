'''
Controls the ASEA IRB6 manipulator by specifying the end effector position and end effector angle

Usage
-----
python3 asea_control_and_simulation.py (config path)
'''

import time
import math
import argparse
import threading
import numpy as np
import numpy.linalg

from init_irb6.controller import ASEAController
from init_irb6.simulation import ASEASimulation


if __name__ == "__main__" : 
	
    # defining the control parameters (for siluation)
	Kp = 1.5
	dt = 0.1
	target_min_distance = 0.05

	# parsing script arguments
	parser = argparse.ArgumentParser()
	parser.add_argument("config_path", help="path to the .json config file")
	args = parser.parse_args()

	# creating the robot simulation and controller
	asea_sim = ASEASimulation(display_mode=True)
	asea_sim.joint_ranges[0] = (0, 0.5 * math.pi)
	asea_controller = ASEAController(args.config_path, selected_axes=[1, 2, 3, 4, 5])

    # defining the target end effector positions
	target_i = 0
	target_points = [(0, 870, 1400), (0, 870, 850), (350, 350, 1400), (650, 650, 850), (870, 0, 1400), (870, 0, 850), (870, 0, 1150)]
 
	target_joint_angles, target_abs_joint_angles = [], []
	for target_point in target_points:
		joint_angles, abs_joint_angles = asea_sim.ik_solution_search(target_point)
		target_joint_angles.append(joint_angles)
		target_abs_joint_angles.append(abs_joint_angles)

	# making sure all computed angles are valid
	angles_valid = True
	for joint_i, joint_angles in enumerate(target_joint_angles):
		angles_valid *= (joint_angles is not None)
		if angles_valid: 
			print(f"Target angles for point {target_points[joint_i]}: {[math.degrees(element) for element in joint_angles]}")
		else:
			print(f"Failed to compute IK solutiuon for point : {target_points[joint_i]}")

	if angles_valid:

		# homing the robotic arm
		print("homing the robot - start")
		asea_controller.home()
		print("homing the robot - end")
		time.sleep(2)

		# main control loop
		while True:

			print(f"target positions : {target_points[target_i]}")

			### reaching the target point in the simulation

			target_reached = False
			while not target_reached:

				# computing the command angular velocities + applying the joint velocities with a discrete time interval
				joint_angular_vels = Kp * asea_sim.ang_diff(target_joint_angles[target_i], asea_sim.joint_angles)
				asea_sim.update_display(asea_sim.joint_angles + joint_angular_vels * dt, target_points=target_points)

				# checking if the effector reached the destination
				angles_d = np.linalg.norm(asea_sim.ang_diff(target_joint_angles[target_i], asea_sim.joint_angles))
				target_reached = angles_d <= target_min_distance

			### controlling the robot to reach the target point (in real life)
			
			# converting the target angles to degrees
			target_abs_joint_angles_deg = [math.degrees(angle) for angle in target_abs_joint_angles[target_i]]

			# moving the waist axis first (looks cooler)
			for axis_i, joint_pos in enumerate(target_abs_joint_angles_deg):
				if (joint_pos is not None) and (asea_controller.axes[axis_i]):
					if asea_controller.axes[axis_i].ax_id == "ax1":
						asea_controller.set_axis_position(axis_i + 1, joint_pos)
			
			# moving all the the other axes
			control_threads = []
			for axis_i, joint_pos in enumerate(target_abs_joint_angles_deg):
				if (joint_pos is not None) and (asea_controller.axes[axis_i]):
					if not asea_controller.axes[axis_i].ax_id == "ax1":
						control_threads.append(threading.Thread(target=asea_controller.axes[axis_i].set_position, args=(joint_pos,)))
						control_threads[-1].start()
			for t in control_threads: t.join()
   
			time.sleep(3)

			# defining the next movement
			target_i = (target_i + 1) % len(target_joint_angles)