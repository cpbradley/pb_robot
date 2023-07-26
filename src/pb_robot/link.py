from collections import namedtuple
import numpy
import pybullet as p
import pb_robot.geometry as geometry

CLIENT = 0

LinkState = namedtuple('LinkState', ['linkWorldPosition', 'linkWorldOrientation',
                                     'localInertialFramePosition', 'localInertialFrameOrientation',
                                     'worldLinkFramePosition', 'worldLinkFrameOrientation'])

class Link(object):
    def __init__(self, body, linkID):
        self.body = body
        self.bodyID = self.body.id
        self.linkID = linkID
        self.parentJointID = linkID # Parent joint and link have same id
        if linkID == -1:
            self.parentJoint = None
        else:
            self.parentJoint = self.body.joints[linkID]
        # have a get joint from id? returns the class? 
        self.base_link = body.base_link

        self.LinkState = LinkState

        #parent_link_from_joint = get_link_parent
        self.link_ancestors = None

    def __repr__(self):
        return self.get_link_name()

    def get_link_name(self):  
        if self.linkID == self.base_link:
            return self.body.get_base_name()
        return self.parentJoint.get_joint_info().linkName.decode('UTF-8')

    def get_link_parent(self):
        if self.linkID == self.base_link:
            return None
        return self.body.all_links[self.linkID]

    def get_link_state(self, kinematics=True, velocity=True):
        # TODO: the defaults are set to False?
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/pybullet.c
        return self.LinkState(*p.getLinkState(self.body.id, self.linkID, 
                                              #computeLinkVelocity=velocity, 
                                              #computeForwardKinematics=kinematics,
                                              physicsClientId=CLIENT))

    def get_com_pose(self): # COM = center of mass
        link_state = self.get_link_state()
        return link_state.linkWorldPosition, link_state.linkWorldOrientation

    def get_link_inertial_pose(self):
        link_state = self.get_link_state()
        return link_state.localInertialFramePosition, link_state.localInertialFrameOrientation

    def get_link_pose(self):
        if self.linkID == self.base_link:
            return self.body.get_pose()
        # if set to 1 (or True), the Cartesian world position/orientation will be recomputed using forward kinematics.
        link_state = self.get_link_state() #, kinematics=True, velocity=False)
        return link_state.worldLinkFramePosition, link_state.worldLinkFrameOrientation

    def get_link_tform(self, worldFrame=False):
        link_worldF = geometry.tform_from_pose(self.get_link_pose())
        link_objF = numpy.dot(numpy.linalg.inv(self.body.get_transform()), link_worldF)
        if worldFrame:
            return link_worldF
        else:
            return link_objF

    def get_link_children(self):
        children = self.body.get_all_link_children()
        for c in children.keys():
            if c.linkID == self.linkID:
                return children[c]
        return []
        #return children.get(self, []) 

    def get_link_ancestors(self):
        if self.link_ancestors is None:
            parent = self.get_link_parent()
            if parent is None:
                self.link_ancestors = []
            else:
                self.link_ancestors = parent.get_link_ancestors() + [parent]
        return self.link_ancestors

    def get_joint_ancestors(self): 
        return [l.parentJoint for l in self.get_link_ancestors()] + [self.parentJoint]

    def get_movable_joint_ancestors(self):
        return self.body.prune_fixed_joints(self.get_joint_ancestors())

    def get_link_descendants(self, test=lambda l: True):
        descendants = []
        for child in self.get_link_children():
            if test(child):
                descendants.append(child)
                descendants.extend(child.get_link_descendants(test=test))
        return descendants

    def get_link_subtree(self, **kwargs):
        return [self] + self.get_link_descendants(**kwargs)

    # def get_local_link_pose(self): #XXX test
    #     # parent_joint = self.body.parent_link_from_joint(self.jointID)
    #     parent_joint = self.parentJoint

    #     #world_child = get_link_pose(body, joint)
    #     #world_parent = get_link_pose(body, parent_joint)
    #     ##return geometry.multiply(geometry.invert(world_parent), world_child)
    #     #return geometry.multiply(world_child, geometry.geometry.invert(world_parent))

    #     # https://github.com/bulletphysics/bullet3/blob/9c9ac6cba8118544808889664326fd6f06d9eeba/examples/pybullet/gym/pybullet_utils/urdfEditor.py#L169
    #     # parent_com = self.body.get_joint_parent_frame(self.jointID)
    #     parent_com = parent_joint.get_joint_parent_frame()
    #     # tmp_pose = geometry.invert(geometry.multiply(self.body.get_joint_inertial_pose(self.jointID), parent_com))
    #     dynamics_info = self.body.get_dynamics_info(linkID=self.linkID)
    #     inert_pose = dynamics_info.local_inertial_pos, dynamics_info.local_inertial_orn
    #     tmp_pose = geometry.invert(geometry.multiply(inert_pose, parent_com))
    #     # tmp_pose = geometry.invert(geometry.multiply(parent_joint.get_joint_inertial_pose(), parent_com))
    #     # parent_inertia = self.body.get_joint_inertial_pose(parent_joint)
    #     parent_inertia = parent_joint.get_joint_inertial_pose()
    #     #return geometry.multiply(parent_inertia, tmp_pose) # TODO: why is this wrong...
    #     _, orn = geometry.multiply(parent_inertia, tmp_pose)
    #     pos, _ = geometry.multiply(parent_inertia, geometry.Pose(parent_com[0]))
    #     return (pos, orn)

    def get_local_link_pose(self):
        # parent_joint = parent_link_from_joint(body, joint)
        import pb_robot
        joint_info = pb_robot.joint.JointInfo(*p.getJointInfo(self.body.id, self.linkID, physicsClientId=CLIENT))
        if self.linkID == -1:
            parent_joint = None
        else:
            parent_joint = joint_info.parentIndex

        #world_child = get_link_pose(body, joint)
        #world_parent = get_link_pose(body, parent_joint)
        ##return multiply(invert(world_parent), world_child)
        #return multiply(world_child, invert(world_parent))

        # https://github.com/bulletphysics/bullet3/blob/9c9ac6cba8118544808889664326fd6f06d9eeba/examples/pybullet/gym/pybullet_utils/urdfEditor.py#L169
        # parent_com = get_joint_parent_frame(body, joint)
        parent_com =  joint_info.parentFramePos, joint_info.parentFrameOrn
        dynamics_info = self.body.get_dynamics_info(linkID=self.linkID)
        inert_pose = dynamics_info.local_inertial_pos, dynamics_info.local_inertial_orn
        tmp_pose = geometry.invert(geometry.multiply(inert_pose, parent_com))
        dynamics_info = self.body.get_dynamics_info(parent_joint)
        parent_inertia = dynamics_info.local_inertial_pos, dynamics_info.local_inertial_orn
        _, orn = geometry.multiply(parent_inertia, tmp_pose)
        pos, _ = geometry.multiply(parent_inertia, geometry.Pose(parent_com[0]))
        return (pos, orn)