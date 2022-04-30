import math
import random
import numpy as np
import numpy.linalg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D

from .config import ConfigurationManager


class ASEAMovementGenerator:
    
    ''' Class for the generation of movements for the ASEA IRB6 robotic arm '''
    
    def __init__(self, config_path):
    
        '''
        Parameters
        ----------
        config_path (string) : path to the json configuration file
        '''
        
        self.simulation = ASEASimulation()
        self.config = ConfigurationManager(config_path)
        
        # loading predefined movements (if required)
        
        self.predefined_movements = self.config["predefined_movements"]
        if self.predefined_movements:
            
            self.movement_index = 0
            self.target_joint_angles, self.target_abs_joint_angles = [], []
            
            # computing the joint angles for the provided ee positions
            ee_positions = self.config["ee_positions"]
            for target_point in ee_positions:
                joint_angles, abs_joint_angles = self.simulation.ik_solution_search(target_point)
                self.target_joint_angles.append(joint_angles)
                self.target_abs_joint_angles.append(abs_joint_angles)

            # making sure all provided ee positions are valid
            for joint_i, joint_angles in enumerate(self.target_joint_angles):
                angles_valid = (joint_angles is not None)
                if not angles_valid: 
                    raise ValueError(f"Failed to compute IK solutiuon for point : {ee_positions[joint_i]}")
            
            
    def __iter__(self):

        if self.predefined_movements:
            self.movement_index = 0 

        return self
    
    
    def __next__(self):
        
        '''
        Returns
        -------
        tuple(float, float, float, float, float) : robot joint angles (in radians)
        tuple(float, float, float, float, float) : robot joint angles (in standard quandrant format and in degrees)
        '''
        
        angles, quadrant_angles = None, None
        
        # pulling the next angles from the predefined list
        if self.predefined_movements:
               
            angles = self.target_joint_angles[self.movement_index]
            quadrant_angles = self.target_abs_joint_angles[self.movement_index]
            
            self.movement_index += 1
            if self.movement_index == len(self.target_joint_angles): self.movement_index = 0
            
        # random movement generation
        else: angles, quadrant_angles = self.simulation.generate_random_angles()
        
        quadrant_angles = [math.degrees(angle) for angle in quadrant_angles]
        return angles, quadrant_angles



class ASEASimulation:

    ''' Class for the simulated control of the ASEA IRB6 robotic arm '''    
    
    # defining the robot's segment lengths (in mm)
    l1 = 450
    l2 = 670
    l3 = 200

    # z axis offset of the elbow joint
    height_offset = 700

    # error threshold for position solutions (%)
    max_pos_error = 0.1

    # number of effector angles to try over the possible range (for ik_solution_search)
    ee_search_n_angles = 20

    # defining the +- rotation range for the robot's joints (in radians)
    # [waist (-172.5, 172.5), shoulder(50, 130), elbow(-40, 10), wrist bend(-90, 90), wrist rotation(-180, 180)]
    joint_ranges = [(-0.9583*math.pi, 0.9583*math.pi),\
                    (0.3333*math.pi, 0.6666*math.pi),\
                    (-0.2222*math.pi, 0.05555*math.pi),\
                    (-0.5*math.pi, 0.5*math.pi),\
                    (-1*math.pi, math.pi)]

    # defining the axes value ranges for the grapgh display
    graph_x_range = 10
    graph_y_range = 10
    graph_z_range = 10

    def __init__(self, scale=0.01, display_mode=False, display_point_grid=False):

        ''' 
        scale (float) : rescale factor for the drawing of the robot's limbs
        display_mode (bool) : if True, display 3D representation of the robot
        display_point_grid : if True, place a grid of points on the graph marking it's extremities
        '''

        self.scale = scale
        self.display_mode = display_mode

        # defining the default pose
        self.default_pos_angles_abs = [0, math.pi/2, -math.pi/2, 0, 0]
        self.joint_angles = self.default_pos_angles_abs

        # building the list of possible effector positions (for exploration in ik_solution_search)
        # prioritizing low end effector angles
        # assuming the interval is symetric around 0
        self.ee_search_angles = [0]
        angle_icr = (self.joint_ranges[-2][1] - self.joint_ranges[-2][0]) / self.ee_search_n_angles
        for i in range(1, self.ee_search_n_angles // 2 + 1):
            self.ee_search_angles.append(0 + i * angle_icr)
            self.ee_search_angles.append(0 + i * -angle_icr)

        # setting up the display and first display
        if self.display_mode:

            # creating the 3D robot graph
            self.fig = plt.figure("ASEA IRB6 simulation")
            self.ax = Axes3D(self.fig)
            self.fig.show()

            # building the point grid
            self.graph_grid_points = []
            if display_point_grid:
                grid_point_increment = self.graph_x_range
                for x_val in range(-self.graph_x_range, self.graph_x_range + grid_point_increment, grid_point_increment):
                    for y_val in range(-self.graph_y_range, self.graph_y_range + grid_point_increment, grid_point_increment):
                        for z_val in range(-self.graph_z_range, self.graph_z_range + grid_point_increment, grid_point_increment):
                            self.graph_grid_points.append((x_val, y_val, z_val))
                        

    def inverse_kinematics(self, target, effector_angle=0):

        '''
        Computes the joint angles required to reach the target position via inverse kinematics

        Parameters
        ----------
        target (tuple(float, float, float)) : target (x,y,z) 3D position coordinates for the end effector
        effector_angle (float) : the effector angle, to maintain
        
        Returns
        -------
        tuple(float, float, float, float, float) : robot joint angles, if movement is possible, else : None
        tuple(float, float, float, float, float) : robot joint angles (in standard quandrant format), if movement is possible, else : None
        '''
        
        solution_joint_angles = []

        try:

            # computing the waist axis angle + random wrist rotation angle
            waist_angle = math.atan2(target[1], target[0])
            wrist_rotation_angle = random.uniform(-1, 1) * math.pi

            # Treating the rest of the robot as a planar (2D) robot offset from the ground
            # Computing the angles for the shoulder, elbow and wrist bending articulations
            # The Z axis becomes the Y axis and the robot's other symetry axis becomes the X axis 

            py_2d = target[2] - self.height_offset
            px_2d = math.sqrt(target[0]**2 + target[1]**2)
            
            # computing the required position for the wrist articulation
            px3_2d = px_2d - self.l3 * math.cos(effector_angle)
            py3_2d = py_2d - self.l3 * math.sin(effector_angle)
        
            # computing the required shoulder and elbow joint angles (usually two possibilities - v1 and v2)

            c2 = (px3_2d**2 + py3_2d**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
            s2_v1 = math.sqrt(1 - c2**2)
            s2_v2 = -1 * s2_v1
            
            elbow_angle_v1 = math.atan2(s2_v1, c2)
            k1_v1 = self.l1 + self.l2 * math.cos(elbow_angle_v1)
            k2_v1 = self.l2 * math.sin(elbow_angle_v1)
            shoulder_angle_v1 = math.atan2(py3_2d, px3_2d) - math.atan2(k2_v1, k1_v1)
            wrist_bend_angle_v1 = effector_angle - shoulder_angle_v1 - elbow_angle_v1

            elbow_angle_v2 = math.atan2(s2_v2, c2)
            k1_v2 = self.l1 + self.l2 * math.cos(elbow_angle_v2)
            k2_v2 = self.l2 * math.sin(elbow_angle_v2)
            shoulder_angle_v2 = math.atan2(py3_2d, px3_2d) - math.atan2(k2_v2, k1_v2)
            wrist_bend_angle_v2 = effector_angle - shoulder_angle_v2 - elbow_angle_v2

            # defining the two potential solutions
            solution_joint_angles.append((waist_angle, shoulder_angle_v1, elbow_angle_v1, wrist_bend_angle_v1, wrist_rotation_angle))
            solution_joint_angles.append((waist_angle, shoulder_angle_v2, elbow_angle_v2, wrist_bend_angle_v2, wrist_rotation_angle))

        except:
            solution_joint_angles = []
        
        # checking the solutions against the joint angle constraints
        # first solution to respect the joint constraints is selected
        final_solution, final_abs_solution = None, None
        for solution_i, solution in enumerate(solution_joint_angles):

            valid_solution, ranges_respected = True, True

            shoulder_abs_angle = solution[1]
            elbow_abs_angle = solution[1] + solution[2]
            wrist_bend_abs_angle = solution[1] + solution[2] + solution[3]
            abs_solution = (solution[0], shoulder_abs_angle, elbow_abs_angle, wrist_bend_abs_angle, solution[-1])
            
            # making sure the solution is valid
            solution_x_2d = self.l1 * math.cos(abs_solution[1]) + self.l2 * math.cos(abs_solution[2]) + self.l3 * math.cos(abs_solution[3])
            solution_y_2d = self.l1 * math.sin(abs_solution[1]) + self.l2 * math.sin(abs_solution[2]) + self.l3 * math.sin(abs_solution[3])
            valid_solution = ((abs(solution_x_2d - px_2d) / abs(px_2d)) <= self.max_pos_error) and ((abs(solution_y_2d - py_2d) / abs(py_2d)) <= self.max_pos_error)

            # making sure the solution joint angles respect the defined ranges
            for joint_i, joint_angle in enumerate(abs_solution):
                ranges_respected *= ((joint_angle >= self.joint_ranges[joint_i][0]) and (joint_angle <= self.joint_ranges[joint_i][1]))

            # stop when a valid and feasable solution is found
            if ranges_respected and valid_solution:
                final_abs_solution = abs_solution
                final_solution = solution_joint_angles[solution_i]
                break

        return final_solution, final_abs_solution


    def ik_solution_search(self, target):

        '''
        Searches for the joint angles required to reach the target position by varying the end effector angle

        Parameters
        ----------
        target (tuple(float, float, float)) : target (x,y,z) 3D position coordinates for the end effector
        
        Returns
        -------
        tuple(float, float, float, float, float) : robot joint angles, if movement is possible, else : None
        tuple(float, float, float, float, float) : robot joint angles (in standard quandrant format), if movement is possible, else : None
        '''

        joint_angles, joint_abs_angles = None, None

        # generating the absolute angle values
        for ee_angle in self.ee_search_angles:
            joint_angles, joint_abs_angles = self.inverse_kinematics(target, ee_angle)
            if joint_angles : break

        return joint_angles, joint_abs_angles


    def generate_random_angles(self):
        
        ''' 
        Generates random angles for all the robot's joints (random movement generation)
        
        Returns
        -------
        tuple(float, float, float, float, float) : robot joint angles
        tuple(float, float, float, float, float) : robot joint angles (in standard quandrant format)
        '''
        
        joint_angles, abs_joint_angles = [], []
        
        # generating the absolute angles
        for joint_range in self.joint_ranges:
            random_angle = abs_joint_angles.append(joint_range[0] + random.uniform(0, joint_range[1] - joint_range[0]))
        
        # generating the kinematic joint angles
        prev_angles = 0
        for i in range(1, 4):
            joint_angle = abs_joint_angles[i] - prev_angles
            prev_angles += joint_angle
            joint_angles.append(joint_angle)
            
        joint_angles = [abs_joint_angles[0]] + joint_angles + [abs_joint_angles[-1]]
        
        return joint_angles, abs_joint_angles
        

    def update_display(self, joint_angles, target_points=[]):

        ''' 
        Updating the robot's position along with the display

        Parameters
        ----------
        joint_angles ([float, float, float, float, float]) : joint angles
        target_points (list(tuple(float, float, float))) : list of 3D points representing the targets to be attained by the end effector
        '''

        self.joint_angles = np.array(joint_angles) % (2 * math.pi)
        
        # making sure display mode is active
        if self.display_mode:

            plt.cla()

            # getting the robot's absolute joint anles (for the mobile joints)
            shoulder_abs_angle = self.joint_angles[1]
            elbow_abs_angle = self.joint_angles[1] + self.joint_angles[2]
            wrist_bend_abs_angle = self.joint_angles[1] + self.joint_angles[2] + self.joint_angles[3]

            # plotting an indicator on the positive part of the X axis (axis coming out of the robot's front)
            self.ax.plot([0, self.graph_x_range], [0, 0], [0, 0], color="#00FF00")

            # plotting the robot's joints

            # defining the two static joints
            base_position = (0,0,0)
            shoulder_position = (0, 0, self.height_offset)

            # computing the elbow joint position
            elbow_z = shoulder_position[2] + self.l1 * math.sin(shoulder_abs_angle)
            elbow_x = self.l1 * math.cos(shoulder_abs_angle) * math.cos(joint_angles[0])
            elbow_y = self.l1 * math.cos(shoulder_abs_angle) * math.sin(joint_angles[0])
            elbow_position = (elbow_x, elbow_y, elbow_z)
        
            # computing the wrist joint position
            wrist_z = elbow_z + self.l2 * math.sin(elbow_abs_angle)
            wrist_x = elbow_x + self.l2 * math.cos(elbow_abs_angle) * math.cos(joint_angles[0])
            wrist_y = elbow_y + self.l2 * math.cos(elbow_abs_angle) * math.sin(joint_angles[0])
            wrist_position = (wrist_x, wrist_y, wrist_z)

            # computing the end effector position
            ee_z = wrist_z + self.l3 * math.sin(wrist_bend_abs_angle)
            ee_x = wrist_x + self.l3 * math.cos(wrist_bend_abs_angle) * math.cos(joint_angles[0])
            ee_y = wrist_y + self.l3 * math.cos(wrist_bend_abs_angle) * math.sin(joint_angles[0])
            ee_position = (ee_x, ee_y, ee_z)

            # building lists of joint coordinates + applying scaling factor
            joint_positions = [base_position, shoulder_position, elbow_position, wrist_position, ee_position]    
            x_list = [element[0] * self.scale for element in joint_positions]
            y_list = [element[1] * self.scale for element in joint_positions]
            z_list = [element[2] * self.scale for element in joint_positions]
 
            # plotting the robot's joints and segments
            self.ax.plot(x_list, y_list, z_list, "o-", color="#0331fc", ms=4, mew=0.5)
            self.ax.plot(x_list[0], y_list[0], z_list[0], 'o', color="#000000")
            self.ax.plot(x_list[-1], y_list[-1], z_list[-1], 'o', color="#000000")

            # plotting the rotation indicators

            # plotting the waist rotation indicator
            waist_indicator_z = 0
            waist_indicator_x = 1.5 * math.cos(joint_angles[0])
            waist_indicator_y = 1.5 * math.sin(joint_angles[0])
            self.ax.plot([base_position[0], waist_indicator_x], [base_position[1], waist_indicator_y], [base_position[2], waist_indicator_z], color="#eb4034")

            # plotting the wrist rotation indicator
            Vxy = np.array([math.cos(joint_angles[0]), math.sin(joint_angles[0]), 0]) 
            vxy = Vxy / numpy.linalg.norm(Vxy)
            vc = np.cross(vxy, [0, 0, 1])
            vz = [0 ,0, 1]
            wrist_indicator_z = np.array(150 * math.sin(joint_angles[-1])) * vz
            wrist_indicator_c = np.array(150 * math.cos(joint_angles[-1])) * vc
            wrist_indicator = [element * self.scale for element in (np.array(ee_position) + wrist_indicator_z + wrist_indicator_c)]            
            self.ax.plot([x_list[-1], wrist_indicator[0]], [y_list[-1], wrist_indicator[1]], [z_list[-1], wrist_indicator[2]], color="#eb4034")

            # plotting the target points
            for target_point in target_points: 
                self.ax.plot(target_point[0] * self.scale, target_point[1] * self.scale, target_point[2] * self.scale, 'gx', color="#24a33b")
 
            # plotting grid points
            for grid_point in self.graph_grid_points: 
                self.ax.plot(grid_point[0], grid_point[1], grid_point[2], 'gx', color="#000000")

            # setting the graph limits
            self.ax.set_xlim(-self.graph_x_range, self.graph_x_range)
            self.ax.set_ylim(-self.graph_y_range, self.graph_y_range)
            self.ax.set_zlim(-self.graph_z_range, self.graph_z_range)
            
            plt.draw()
            plt.pause(0.0001)
        

    @staticmethod
    def ang_diff(theta1, theta2):

        """ Returns the difference between two angles in the range -pi to +pi """
    
        theta1 = np.array(theta1)
        theta2 = np.array(theta2)
        return (theta1 - theta2 + np.pi) % (2 * np.pi) - np.pi