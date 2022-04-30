import time
import argparse

from gpiozero import Button, LED

from Phidget22.Phidget import *
from Phidget22.ErrorCode import *
from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.Devices.DigitalInput import *

from .config import ConfigurationManager


class MovementSensor:

    ''' Interface for the Phidgets 1111 motion sensor '''

    def __init__(self, config_path):

        '''
        config_path (str) : path to the configuration file
        '''

        self.config = ConfigurationManager(config_path)

        self.movement_wait_time = self.config["ms_wait_time"]
        self.volt_ratio_trigger = self.config["ms_volt_ratio_trigger"]


        # defining the voltage ratio change call back
        def on_voltage_ratio_change(ch, voltage_ratio):     

            if (voltage_ratio >= self.volt_ratio_trigger) and\
               (not ch.movement_flag) and (time.time() >= ch.time_treshold):
                ch.movement_flag = True
                ch.time_treshold = time.time() + self.movement_wait_time


        # defining the channel for the movement sensor (voltage ratio)
        self.ch = VoltageRatioInput()
        self.ch.setDeviceSerialNumber(self.config["hub_sn"])
        self.ch.setHubPort(self.config["ms_port"])
        self.ch.setIsHubPortDevice(True)
        self.ch.setOnVoltageRatioChangeHandler(on_voltage_ratio_change)

        # defining the time management vars
        self.ch.time_treshold = 0
        self.ch.movement_flag = False

        # connecting to the channel
        self.ch.openWaitForAttachment(5000)
        time.sleep(1)


    def __del__(self):
        self.ch.close()


    def check_movement(self):

        ''' 
        Returns True if a movement was detected 
        ** when movements are detected, the flag will stay active until checked
        '''

        flag = self.ch.movement_flag
        if self.ch.movement_flag: self.ch.movement_flag = False

        return flag


    def reset(self):

        self.ch.movement_flag = False
        self.ch.time_treshold = time.time() + self.movement_wait_time



class LimitSwitch():

    ''' Interface for the robot's limit switches connected on the Raspberry pi's GPIO '''

    def __init__(self, pin_number):

        '''
        pin_number (int) : ID of the pin used from the RPI4's GPIO
        '''

        self.switch = Button(pin_number)
        self.update_state()


    def check_switch(self):

        ''' Returns true when the state of the switch changed '''

        switch_state = self.switch.is_pressed
        state_changed = not switch_state == self.state
        self.state = switch_state

        return state_changed


    def update_state(self):

        self.state = self.switch.is_pressed



class LightIndicator:

    ''' Interface for the retro green and red lights which come with the robot '''

    def __init__(self, red_pin_number, green_pin_number):

        ''' 
        red_pin_number (int) : ID of the pin used for the red light from the RPI4's GPIO
        green_pin_number (int) : ID of the pin used for the green light from the RPI4's GPIO
        '''
        
        self.red_light = LED(red_pin_number)
        self.green_light = LED(green_pin_number)


    def set_red_light_state(self, state):

        if state: self.red_light.on()
        else: self.red_light.off()


    def set_green_light_state(self, state):

        if state: self.green_light.on()
        else: self.green_light.off()



def test_movement_sensor(config_path):

    movement_sensor = MovementSensor(config_path)

    while True:

        if movement_sensor.check_movement():
            print("movement detected")

        time.sleep(1)



def test_limit_switch():

    limit_switch = LimitSwitch(2)

    while True:

        if limit_switch.check_switch():
            print("Limit switch triggered")

        time.sleep(0.2)



def test_robot_lights():

    light_indicator = LightIndicator(17, 27)

    while True:

        light_indicator.set_green_light_state(True)
        light_indicator.set_red_light_state(True)
        time.sleep(5)

        light_indicator.set_green_light_state(False)
        light_indicator.set_red_light_state(True)
        time.sleep(5)

        light_indicator.set_green_light_state(True)
        light_indicator.set_red_light_state(False)
        time.sleep(5)

        light_indicator.set_green_light_state(False)
        light_indicator.set_red_light_state(False)
        time.sleep(5)



if __name__ == "__main__":

    # parsing script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("config_path", help="path to the .json config file")
    args = parser.parse_args()

    # testing the movement sensor
    #test_movement_sensor(args.config_path)

    # testing the limit switch
    # test_limit_switch()

    # testing the robot's lights
    test_robot_lights()