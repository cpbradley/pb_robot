import random
import time
import numpy
import pybullet as p
import pb_robot
from transformations import translation_matrix, rotation_matrix, inverse_matrix, concatenate_matrices

class Husky(pb_robot.body.Body):
    '''Create all the functions for controlling the Boston Dynamics Spot'''
    def __init__(self):
        '''Generate the body and establish the other classes'''
        self.urdf_file = 'models/spot_description/spot.urdf'

        with pb_robot.helper.HideOutput(): 
            with pb_robot.utils.LockRenderer():
                self.id = pb_robot.utils.load_model(self.urdf_file, fixed_base=True)
        pb_robot.body.Body.__init__(self, self.id)

        self.eeName = 'arm_link_wr1'
        self.arm_joint_names = ['arm_sh0', 'arm_sh1', 'arm_el0', 'arm_el1', 'arm_wr0', 'arm_wr1']
        self.arm_joints = [self.joint_from_name(n) for n in self.arm_joint_names]

        # self.base_joint_names = ['x', 'y', 'theta']
        self.base_joint_names = ['base_joint']
        self.base_joints =[self.joint_from_name(n) for n in self.base_joint_names]

        self.base_joint = self.base_joints[0]

        # self.body_joint_names = ['_joint']
        # self.body_joints =[self.joint_from_name(n) for n in self.body_joint_names]

        # self.hand = SpotHand(self.id, eeFrame=self.link_from_name(self.eeName))
        # self.arm = Manipulator(self.id, self.arm_joints, self.hand, self.eeName)
        self.hand = None
        self.arm = None