#!/usr/bin/env python
# encoding: utf-8

import threading
import time
import sys
sys.path.append("../")
import csv
from RCPContext.RCPContext import RCPContext
from RCPControl.OrientalMotor import OrientalMotor
from RCPControl.MaxonMotor import MaxonMotor
from RCPControl.InfraredReflectiveSensor import InfraredReflectiveSensor
from RCPControl.ForceSensor import ForceSensor
#from EmergencySwitch import EmergencySwitch
TRANS_EFFICIENT = float(2*16)/(529*60)

class Dispatcher(object):
    """
        description:this class plays an role in th command and control of the interventional robot which includes:
                         -- the control of GPIOs of the raspberryPi which connet motors, sensors and grippers
                         -- the distribution of tasks in different threads
                         -- the command and control of the actions of interventional robot in surgery   
	author:Cheng WANG
    """
    def __init__(self, context, local_mode=0):
        self.context = context

	    # ---------------------------------------------------------------------------------------------
        # initialisation
        # ---------------------------------------------------------------------------------------------
        self.flag = True
        self.cptt = 0
        self.global_state = 0
        self.needToRetract = False
        self.draw_back_guidewire_curcuit_flag = True
        self.number_of_cycles = 0
        self.force_flag = False
	    # ---------------------------------------------------------------------------------------------
	    # execution units of the interventional robot
	    # ---------------------------------------------------------------------------------------------
        self.agencyMotor = MaxonMotor(2, "EPOS2", "MAXON SERIAL v2", "USB", "USB0", 1000000)
        self.particleMotor = MaxonMotor(1, "EPOS2", "MAXON SERIAL V2", "USB", "USB1", 1000000)
	
	    # ---------------------------------------------------------------------------------------------
        # TODO : haptic sensors
        # ---------------------------------------------------------------------------------------------
        # self.hapticSensor = HapticSensor()

        # ---------------------------------------------------------------------------------------------
        # speed parameters
        # ---------------------------------------------------------------------------------------------
        self.speedProgress = 1000 
        self.speedRotate = 60
        self.speedCatheter =10
        self.rotateTime = 180/self.speedRotate

        self.pos_speed = 5
        self.position_cgf = 2
        self.position_cgb = -100

	    # ---------------------------------------------------------------------------------------------
        # real time task to parse commandes in context
        # ---------------------------------------------------------------------------------------------
        if local_mode == 0:
       	    self.dispatchTask = threading.Thread(None, self.do_parse_commandes_in_context)
       	    self.dispatchTask.start()

        self.force_sensor = ForceSensor("/dev/ttyusb_force", 9600, 8, 'N', 1)
        self.obtain_force_task = threading.Thread(None, self.storing_force_data)
        self.force_flag = True
        self.obtain_force_task.start()


    def set_global_state(self, state):
	    self.global_state = state

    def do_parse_commandes_in_context(self):
    	# determine system's status and start to decode or to close devices
        while self.flag:
            if not self.context.get_system_status():
                self.agencyMotor.close_device()
                self.particleMotor.close_device()
                self.flag = False
                print("system terminated")
            else:
                self.decode()
            time.sleep(0.02)
	    
    def decode(self):
        # decode messages in the sequence and performe operations
	    # ---------------------------------------------------------------------------------------------
        # catheter execution case
        # ---------------------------------------------------------------------------------------------
        if self.context.get_catheter_move_instruction_sequence_length() > 0:
            msg = self.context.fetch_latest_catheter_move_msg()
            if self.draw_back_guidewire_curcuit_flag == False:
                return 
            realTargetVelocity = int((10.0*msg.get_motor_speed())/(200*TRANS_EFFICIENT))
            if msg.get_motor_orientation() == 1: # rm_move_to_position(400,8000*50)
                # self.frontNeedleMotor.rm_move(-realTargetVelocity)
                return
            elif msg.get_motor_orientation() == 0:
            # self.frontNeedleMotor.rm_move_to_position(-msg.get_motor_speed(), 8000)
                self.agencyMotor.rm_move(realTargetVelocity)
                return
        elif self.context.get_guidewire_progress_instruction_sequence_length() > 0:
            msg = self.context.fetch_latest_guidewire_progress_move_msg()
            if msg.get_motor_orientation() == 1: 
                self.particleMotor.rm_move(-msg.get_motor_speed() * 5)
                return
            elif msg.get_motor_orientation() == 0:
                self.particleMotor.rm_move(msg.get_motor_speed() * 5)
                return

    def storing_force_data(self):
        path = "hapticForce.csv"
        data = list()
        while self.force_flag:
            force = self.force_sensor.aquireForce()
            timestamps = time.time()
            localtime = time.localtime(timestamps)
            tmpdata = list()
            m_second = int((timestamps-int(timestamps))*1000)
            tmpdata.append(localtime.tm_hour)
            tmpdata.append(localtime.tm_min)
            tmpdata.append(localtime.tm_sec)
            tmpdata.append(m_second)
            #tmpdata.append(self.agencyMotor.profile_position())
            tmpdata.append(force)
            data.append(tmpdata)
            if len(data) >= 100:
                for var in data:
                    with open(path, 'a+') as f:
                        csv_writer = csv.writer(f)
                        csv_writer.writerow(var)
                        f.write('\r\n')
                del data[0:100]
            time.sleep(0.020)

    def stop_storing_data(self):
        self.force_flag = False


dispatcher = Dispatcher(1, 1)
print("please enter!")
type_input = input()
type_input_array = type_input.split(" ")
relative_position = int(type_input_array[1])
speed = int(type_input_array[0])
sleeptime = relative_position / speed
pulse_speed = int((speed / 2)*33.3*60)
relative_position_qc = int((relative_position / 2)*33.3*4000)
dispatcher.agencyMotor.rm_move_to_position(pulse_speed, -1*relative_position_qc)
print("forward")
time.sleep(sleeptime+10)
dispatcher.agencyMotor.rm_move_to_position(pulse_speed, relative_position_qc)
print("backward")
time.sleep(sleeptime+10)
dispatcher.stop_storing_data()

