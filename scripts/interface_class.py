#!/usr/bin/python

import rospy

import numpy as np
import rospy
import std_msgs.msg
from  rospy.numpy_msg import numpy_msg
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import os


# from motormodule import MotorModuleController
# from can_msgs import Frame
class Interface:
    
    def __init__(self):
        # Atributos
        # ****************************** Pata frente izquierda *****************
        # LF_ABD_motor = MotorModuleController(1)
        # LF_FLX1_motor = MotorModuleController(2)
        # LF_FLX2_motor = MotorModuleController(3)
        # ****************************** Pata frente derecha *****************
        # RF_ABD_motor = MotorModuleController(4)
        # RF_FLX1_motor = MotorModuleController(5)
        # RF_FLX2_motor = MotorModuleController(6)
        # # ****************************** Pata trasera izquierda *****************
        # LR_ABD_motor = MotorModuleController(7)
        # LR_FLX1_motor = MotorModuleController(8)
        # LR_FLX2_motor = MotorModuleController(9)
        # # ****************************** Pata trasera derecha **************
        # RR_ABD_motor  = MotorModuleController(A) #10
        # RR_FLX1_motor = MotorModuleController(B) #11
        # RR_FLX2_motor = MotorModuleController(C) #12

        # joints_state = JointState()
        # joints_state.header = std_msgs.msg.Header()
        # joints_state.name = ['LF_ABD','LF_HF1','LF_FF2','LF_FT_FX','RF_ABD','RF_HF1','RF_FF2','RF_FT_FX','LR_ABD','LR_HF1','LR_FF2','LR_FT_FX','RR_ABD','RR_HF1','RR_FF2','RR_FT_FX']
        # joints_state.position = [0,0,0,0,0,0,0,0,0,0,0,0]
        # joints_state.velocity = []
        # joints_state.effort = []
        self.init_can()


        self.joints_trayec = JointTrajectory

        # Subscribers
        rospy.Subscriber("/joint_group_position_controller/command", JointTrajectory, self.control_callback, queue_size=10 )
        rospy.Subscriber("/joints_calibrator", JointState, self.control_callback, queue_size=10 )
        # rospy.Subscriber("topic", Type, callback)

        # Publisher
        # self.current_angle = rospy.Publisher('/joint_states', JointState, queue_size=10)

    #--------------------------------------------------------------------------------------#
    # Callback o interrupcion
    def control_callback(self, champ):
        #--------------------------------------------------------------------------------#
        # MTH from base to camera
      
        # Transform obtained from ROS
        try:
            # champ
            # self.joints_trayec = champ
            # LF_ABD = champ.points[0].positions[0]
            # LF_FLX1 = champ.points[0].positions[1]
            # LF_FLX2 = champ.points[0].positions[2]
            # print('LF_ABD_motor',self.joints_trayec)
            # print('LF_ABD_motor',LF_ABD_motor)
            # print('LF_FLX1_motor',LF_FLX1_motor)
            # print('LF_FLX2_motor',LF_FLX2_motor)
            self.joints_trayec = champ
            LF_ABD = champ.position[0]
            print('LF_ABD_motor',self.joints_trayec)
        except :
            rospy.logwarn("Error reading joint_group_position_controller")
            return
        
        LF_ABD_motor = MotorModuleController(1)
        LF_FLX1_motor = MotorModuleController(2)
        LF_FLX2_motor = MotorModuleController(3)

        LF_FLX2_motor.send_command(LF_ABD, 0, kp, kd, i_ff)
        

    def init_can(self):
        os.system('sudo ip link set can0 type can bitrate 1000')
        os.system('sudo ifconfig can0 up')
        # self.can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan',is_extended_id=False)
        # self.rx_msg = None
        print('connected to motor module controller')
        # joints_state.position = champ.position()

        # self.current_angle.publish(joints_state)