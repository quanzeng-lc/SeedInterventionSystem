#! /usr/bin/env python3
# encoding:utf-8

import sys
sys.path.append("../")
from RCPControl.MaxonMotor import MaxonMotor

class AgencyMotor(MaxonMotor):

    def __init__(self):
        super(AgencyMotor, self).__init__(self, RMNodeId, pDeviceName, pProtocolStackName, pInterfaceName, pPortName, lBaudrate)
        self.gear = 33
        self.lead = 2  # mm
        self.position_resolution = 4000  # qc
        self.profile_velocity = 0  # mm/s
        self.profile_relative_position = 0  # mm

    def profile_parameter_reset(self):
        self.profile_relative_position = 0
        self.profile_velocity = 0

    def open_device(self):
        super(AgencyMotor, self).open_device()

    def set_profile_velocity(self, profile_velocity):
        self.profile_velocity = profile_velocity

    def set_profile_relative_position(self, profile_relative_position):
        self.profile_relative_position = profile_relative_position

    def set_profile_mode_parameter(self, profile_velocity, profile_relative_position):
        self.set_profile_velocity(profile_velocity)
        self.set_profile_relative_position(profile_relative_position)

    def profile_position_move(self):
        profile_position_motor = (self.profile_velocity / self.lead) * self.gear * self.position_resolution
        profile_velocity_motor = (self.profile_velocity / self.lead) * self.gear * 60
        self.rm_move_to_position(profile_velocity_motor, profile_position_motor)
        self.profile_parameter_reset()

    def profile_velocity_move(self):
        profile_velocity_motor = (self.profile_velocity / self.lead) * self.gear * 60
        self.rm_move(profile_velocity_motor)