import time
import datetime
import threading

import logging
import logging.handlers

from Phidget22.Phidget import *
from Phidget22.ErrorCode import *
from Phidget22.Devices.DCMotor import *
from Phidget22.Devices.MotorPositionController import *

from .gpio import LimitSwitch
from .config import ConfigurationManager


class AxisController:
    
    ''' Interface for the implementation of axis controls on the ASEA irb6 robotic manipulator '''

    def __init__(self, ax_id, config):

        '''
        Parameters
        ----------
        ax_id (string) : unique identifier for the axis. Used to pull the proper configuration params.
        config (ConfigurationManager) : reference to the ASEAController's config manager 
        '''

        self.ax_id = ax_id
        self.config = config

        self.controller = None
        self.error_state = False

        # defining the phidget controller error handler
        def on_error(controller, code, description):
            details = ErrorEventCode.getName(code) + " - " + str(description)
            self.force_stop(details, True)
        self.phidget_error_handler = on_error

        # loading common parameters
        self.axis_type = self.config[self.ax_id + "_type"]
        self.home_direction = self.config[self.ax_id + "_home_direction"]
        self.home_value_switch = self.config[self.ax_id + "_home_value_switch"]
        self.home_value_no_switch = self.config[self.ax_id + "_home_value_no_switch"]

        # defining the homing mechanism for the axis (limit switch or not)
        self.limit_switch = None
        if self.config[self.ax_id + "_limit_switch_pin"] is not None:
            self.limit_switch = LimitSwitch(self.config[self.ax_id + "_limit_switch_pin"])

        # setting up error message logging
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)
        log_handler = logging.handlers.WatchedFileHandler(self.config["log_file_path"])
        log_handler.setFormatter(logging.Formatter('%(name)s - %(levelname)s - %(message)s'))
        logger.addHandler(log_handler)


    def __del__(self):

        try: 
            if self.axis_type == "position": self.controller.setEngaged(False)
            self.controller.close()
        except: pass


    def force_stop(self, details="", error_state=False):

        ''' 
        Emergency stop for the axis

        Parameters
        ---------
        details (string) : String detailing the reason for the foreceful stop
        error_state (bool) : Sets the error state flag for ASEAAxis 
        '''

        self.error_state = error_state

        if self.axis_type == "velocity":
            self.controller.setTargetVelocity(0)
        elif self.axis_type == "position":
            self.controller.setEngaged(False)

        time_str = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S')
        logging.error(f"{time_str} - ASEAAxis - axis : {self.ax_id} was forced to stop : {str(details)}")


    def set_position(self, position):

        ''' 
        The axis motor will try to reach the provided position

        Parameters
        ----------
        position (float) : required angular position of the axis motor (in degrees)
        '''

        pass


    def send_home(self):

        ''' Setting the axis position to it's predefined home '''

        pass



class AxisPositionController(AxisController):

    ''' Class for the (position) control of a single axis of the ASEA irb6 robotic manipulator '''
    
    def __init__(self, ax_id, config):

        '''
        Parameters
        ----------
        ax_id (string) : unique identifier for the axis. Used to pull the proper configuration params.
        config (ConfigurationManager) : reference to the ASEAController's config manager 
        '''

        super(AxisPositionController, self).__init__(ax_id, config)

        self.curr_position_enc = None
        self.curr_position_deg = None

        self.max_velocity = self.config[self.ax_id + "_max_velocity"]
        self.movement_delay = self.config[self.ax_id + "_movement_delay"]

        # loading encoder position to degrees position conversion parameters
        self.min_angle = self.config[self.ax_id + "_min_angle"]
        self.max_angle = self.config[self.ax_id + "_max_angle"]
        self.min_angle_position = self.config[self.ax_id + "_min_angle_position"]
        self.max_angle_position = self.config[self.ax_id + "_max_angle_position"]
        self.enc_ang_slope = (self.max_angle_position - self.min_angle_position) / (self.max_angle - self.min_angle)
        self.ang_enc_slope = (self.max_angle - self.min_angle) / (self.max_angle_position - self.min_angle_position) 

        # defining the valid encoder position value range
        enc_position_tolerance = abs(self.max_angle_position - self.min_angle_position) * 0.05
        self.enc_position_range = [min(self.min_angle_position, self.max_angle_position) - enc_position_tolerance,\
                                   max(self.min_angle_position, self.max_angle_position) + enc_position_tolerance]

        self.configure_position_controller()


    def configure_position_controller(self):

        ''' Configures the axis position control with the configuration '''

        # opening and attaching the position control channel
        self.controller = MotorPositionController()
        self.controller.setDeviceSerialNumber(self.config["hub_sn"])
        self.controller.setHubPort(self.config[self.ax_id + "_hub_port"])
        self.controller.openWaitForAttachment(5000)
        
        # configuring data handling
        self.controller.setOnErrorHandler(self.phidget_error_handler)        

        # configuring the position control channel
        self.controller.setKp(self.config[self.ax_id + "_kp"])
        self.controller.setKi(self.config[self.ax_id + "_ki"])
        self.controller.setKd(self.config[self.ax_id + "_kd"])
        self.controller.setDeadBand(self.config[self.ax_id + "_dead_band"])
        self.controller.setCurrentLimit(self.config[self.ax_id + "_current_limit"])
        self.controller.setVelocityLimit(self.config[self.ax_id + "_max_velocity"])
        self.controller.setAcceleration(self.config[self.ax_id + "_acceleration"])

        # defining the initial position as 0 (this will help with homing)
        self.controller.addPositionOffset(-1 * self.controller.getPosition())


    def __del__(self):
        super(AxisPositionController, self).__del__() 


    def set_position(self, position):

        ''' 
        The axis motor will try to reach the provided position

        Parameters
        ----------
        position (float) : required angular position of the axis motor (in degrees)
        '''

        # checking the validity of the current position (in encoder units) and the requested position (in degrees)
        current_position_enc = self.controller.getPosition()
        if (position is not None) and (position <= self.max_angle and position >= self.min_angle) and\
           current_position_enc >= self.enc_position_range[0] and current_position_enc <= self.enc_position_range[1]:
           
            if not position == self.curr_position_deg:

                # keeping track of the previous position
                self.curr_position_deg = position
                self.curr_position_enc = current_position_enc

                # transforming the angular position to encoder format (interpolation)
                enc_pos = self.angle_to_encoder(position)

                # sending the axis to the target position
                self.controller.setEngaged(True)
                self.controller.setTargetPosition(enc_pos)
                time.sleep(self.movement_delay)
           
        else:
            time_str = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S')
            error_str = f"{time_str} - AxisPositionController - axis : {self.ax_id} 'set_position' function : provided position\
                or current position is out of bounds."
            logging.error(error_str)
            raise ValueError(error_str)


    def send_home(self):

        ''' Setting the axis position to it's predefined home '''

        # homing approach when the axis has a limit switch
        # for some joints (ex. the shoulder) it's not a great idea to blindly try to find the home position
        # via a sweep because the articulation can get stuck if it extends to much in the wrong direction. Same goes
        # for when the robot just got done with homing.
        if (self.limit_switch is not None) and (self.curr_position_enc is not None) and (not self.curr_position_enc == self.home_direction):

            # temporarily replacing the position control by a velocity control
            self.controller.close()
            self.controller = DCMotor()
            self.controller.setDeviceSerialNumber(self.config["hub_sn"])
            self.controller.setHubPort(self.config[self.ax_id + "_hub_port"])
            self.controller.openWaitForAttachment(5000)
            self.controller.setOnErrorHandler(self.phidget_error_handler)
            self.controller.setAcceleration(1)
            self.controller.setCurrentLimit(self.config[self.ax_id + "_current_limit"])

            # calculating the movement direction to get to the limit switch
            direction = 1
            if (self.home_direction - self.curr_position_enc) < 0: direction = -1

            # moving towards the home position until the limit switch is hit
            self.limit_switch.update_state()
            self.controller.setTargetVelocity(0.4 * direction)
            while(not self.limit_switch.check_switch()): time.sleep(0.1)
            self.controller.setTargetVelocity(0)
            time.sleep(1)

            # reconfiguring the position controller
            self.controller.close()
            self.configure_position_controller()

            # once the axis is home, set the current position to the (switch) home value
            self.controller.addPositionOffset(-1 * self.controller.getPosition() + self.home_value_switch)

            # setting the new angular and encoder positions
            self.curr_position_enc = self.home_value_switch
            self.curr_position_deg = self.encoder_to_angle(self.home_value_switch)

        # homing approcah when the axis has no limit switch (sending the axis to one of it's extremes)
        else:

            # sending the axis to it's defined home position
            self.controller.setEngaged(True)
            self.controller.setTargetPosition(self.home_direction)
            time.sleep(self.movement_delay)

            # once the axis is home, set the current position to the (no switch) home value
            self.controller.addPositionOffset(-1 * self.controller.getPosition() + self.home_value_no_switch)

            # setting the new angular and encoder positions
            self.curr_position_enc = self.home_value_no_switch
            self.curr_position_deg = self.encoder_to_angle(self.home_value_no_switch)

        # after homing, sending the axis to it's starting position
        if self.config[self.ax_id + "_starting_pos_deg"] is not None:
            self.set_position(self.config[self.ax_id + "_starting_pos_deg"])
        elif self.config[self.ax_id + "_starting_pos_enc"] is not None:
            self.controller.setEngaged(True)
            self.controller.setTargetPosition(self.config[self.ax_id + "_starting_pos_enc"])
            time.sleep(self.movement_delay)


    def encoder_to_angle(self, encoder_val):

        ''' Converting the axis encoder units into angles '''
        
        return (encoder_val - self.min_angle_position) * self.ang_enc_slope + self.min_angle


    def angle_to_encoder(self, angle_val):
        
        ''' Converting the axis angle into encoder units '''

        return int((angle_val - self.min_angle) * self.enc_ang_slope + self.min_angle_position)



class AxisVelocityController(AxisController):

    ''' Class for the (velocity) control of a single axis of the ASEA irb6 robotic manipulator '''

    def __init__(self, ax_id, config):

        '''
        Parameters
        ----------
        ax_id (string) : unique identifier for the axis. Used to pull the proper configuration params.
        config (ConfigurationManager) : reference to the ASEAController's config manager 
        '''

        super(AxisVelocityController, self).__init__(ax_id, config)

        # loaading general movement parameters
        self.angular_position = self.config[self.ax_id + "_default_pos"]
        self.min_angle = self.config[self.ax_id + "_min_angle"]
        self.max_angle = self.config[self.ax_id + "_max_angle"]

        # loading additional homing parameters
        self.home_time = self.config[self.ax_id + "_home_time"]
        self.start_position = self.config[self.ax_id + "_starting_pos"]

        self.movement_velocity = self.config[self.ax_id + "_movement_velocity"]
        self.real_movement_velocity = self.config[self.ax_id + "_real_movement_velocity"]

        # opening and attaching the position control channel
        self.controller = DCMotor()
        self.controller.setDeviceSerialNumber(self.config["hub_sn"])
        self.controller.setHubPort(self.config[self.ax_id + "_hub_port"])
        self.controller.openWaitForAttachment(5000)
        self.controller.setOnErrorHandler(self.phidget_error_handler)
        self.controller.setCurrentLimit(self.config[self.ax_id + "_current_limit"])
        self.controller.setAcceleration(self.config[self.ax_id + "_acceleration"])


    def set_position(self, position):

        ''' 
        The axis motor will try to reach the provided position

        Parameters
        ----------
        position (float) : required angular position of the axis motor (in degrees)
        '''

        if (position is not None) and (position <= self.max_angle and position >= self.min_angle):

            if not position == self.angular_position: 

                # computing the time to get to the required position
                ang_diff = position - self.angular_position
                dirrection = (ang_diff / abs(ang_diff))
                required_time = abs(ang_diff) / self.real_movement_velocity

                # setting the new position
                self.angular_position = position

                # moving the axis the for the required amount of time
                self.controller.setTargetVelocity(self.movement_velocity * dirrection)
                time.sleep(required_time)
                self.controller.setTargetVelocity(0)

        else:
            time_str = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S')
            error_str = f"{time_str} - AxisVelocityController - axis : {self.ax_id} 'set_position' function : provided position is out of bounds."
            logging.error(error_str)
            raise ValueError(error_str)


    def send_home(self):

        ''' Setting the axis position to it's predefined home '''

        # homing approach when the axis has a limit switch
        if (self.limit_switch is not None) and (self.angular_position is not None):

            # making sure we are not at home
            if not self.angular_position == self.home_value_switch:

                # calculating the movement direction to get to the limit switch
                direction = 1
                if (self.home_value_switch - self.angular_position) < 0: direction = -1

                # moving towards the home position until the limit switch is hit
                
                self.limit_switch.update_state()
                self.controller.setTargetVelocity(0.4 * direction)
                
                time_inc = 0.1
                error_state = False
                elapsed_home_time = 0
                while not self.limit_switch.check_switch() and not error_state:
                    time.sleep(time_inc)
                    if elapsed_home_time > self.home_time: error_state = True
                    else: elapsed_home_time += time_inc
                    
                self.controller.setTargetVelocity(0)
                
                if not error_state: self.angular_position = self.home_value_switch
                else:
                    time_str = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S')
                    error_str = f"{time_str} - AxisPositionController - axis : {self.ax_id} failed it's homing sequence."
                    logging.error(error_str)
                    raise ValueError(error_str)            

        # homing approach when the axis has no limit switch
        else:

            direction = self.home_direction / abs(self.home_direction)
            self.controller.setTargetVelocity(self.movement_velocity * direction)
            time.sleep(self.home_time)
            self.controller.setTargetVelocity(0)

            self.angular_position = self.home_value_no_switch

        # sending the axis at the start position
        self.set_position(self.start_position)



class ASEAController:

    ''' Class for the control of the ASEA irb6 robotic manipulator (all of the axes) '''

    n_axes = 5

    def __init__(self, config_path, selected_axes=[1,2,3,4,5]):

        '''
        Parameters
        ----------
        config_path (string) : path to the json configuration file
        slected_axes (list(int)) : index of the selected axes (axis count starts from 1)
        '''

        self.selected_axes = selected_axes
        self.config = ConfigurationManager(config_path)

        # creating the control instance for each axis
        self.axes = []
        for axis_i in range(1, self.n_axes + 1):

            if axis_i in self.selected_axes:
                
                ax_id = "ax" + str(axis_i)
                axis_type = self.config[ax_id + "_type"]
                if axis_type == "position":
                    self.axes.append(AxisPositionController(ax_id, self.config))
                elif axis_type == "velocity":
                    self.axes.append(AxisVelocityController(ax_id, self.config))

            else: self.axes.append(None)


    def force_stop(self, details=""):

        ''' 
        Emergency stop for all axes of the arm

        Parameters
        ---------
        details (string) : String detailing the reason for the foreceful stop
        '''

        for axis in self.axes:
            if axis: axis.force_stop(details)

        time_str = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S')
        logging.error(f"{time_str} - ASEAController was forced to stop : {str(details)}")


    def set_axis_position(self, index, position):

        ''' 
        Sets the angular position of the specified axis
        This can only be done when the axis is available, ie : not currently moving

        Parameters
        ----------
        index (int) : axis index (index starts at 1 to match the ASEA documentation)
        position (float) : angular position in degrees
        '''

        if (index <= self.n_axes) and (index > 0):

            target_axis = self.axes[index - 1]
            if target_axis:

                if not target_axis.error_state:
                    target_axis.set_position(position)

        else:
            raise ValueError("The specified axis index is invalid") 

    
    def home(self):

        ''' Homing all the axes of the arm '''
        
        home_threads = []
        for i, axis in enumerate(self.axes):
            home_threads.append(threading.Thread(target=axis.send_home))
            home_threads[i].start()
            
        for t in home_threads: t.join()