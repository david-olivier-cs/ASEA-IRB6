import math
import numpy as np

from init_irb6.simulation import ASEASimulation


if __name__ == "__main__":

    # defining the control parameters
    Kp = 1.5
    dt = 0.1
    target_min_distance = 0.05

    # creation the simulation 
    asea_sim = ASEASimulation(display_mode=True, display_point_grid=False)
    asea_sim.joint_ranges[0] = (0, 0.5 * math.pi)

    # calculating joint angles for target positions + building a list of target positions
    target_i = 0
    target_joint_angles = []
    target_points = [(0, 870, 1400), (0, 870, 850), (350, 350, 1400), (650, 650, 850)]
    for target_point in target_points:
        joint_angles, _ = asea_sim.ik_solution_search(target_point)
        target_joint_angles.append(joint_angles)
        
    print(f"joint angles : {target_joint_angles}")
    
    # making sure all computed angles are valid
    angles_valid = True
    for joint_i, joint_angles in enumerate(target_joint_angles):
        angles_valid *= (joint_angles is not None)
        if angles_valid: 
            print(f"Target angles for point {target_points[joint_i]}: {[math.degrees(element) for element in joint_angles]}")
        else:
            print(f"Failed to compute IK solutiuon for point : {target_points[joint_i]}")

    if angles_valid:
    
        # main control loop
        while True:

            target_reached = False
            while not target_reached:

                # computing the command angular velocities + applying the joint velocities with a discrete time interval
                joint_angular_vels = Kp * asea_sim.ang_diff(target_joint_angles[target_i], asea_sim.joint_angles)
                asea_sim.update_display(asea_sim.joint_angles + joint_angular_vels * dt, target_points=target_points)

                # checking if the effector reached the destination
                angles_d = np.linalg.norm(asea_sim.ang_diff(target_joint_angles[target_i], asea_sim.joint_angles))
                target_reached = angles_d <= target_min_distance

            # selection of the next target position
            target_i = (target_i + 1) % len(target_joint_angles)