"""supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import copy
from controller import Supervisor
import numpy as np



supervisor = None
robot_node = None
target_node = None

def init_supervisor():
    global supervisor, robot_node, target_node

    # create the Supervisor instance.
    supervisor = Supervisor()

    # do this once only
    root = supervisor.getRoot() 
    root_children_field = root.getField("children") 
    robot_node = root_children_field.getMFNode(5) 
    target_node = root_children_field.getMFNode(-1) 
    start_translation = copy.copy(robot_node.getField("translation").getSFVec3f())
    start_rotation = copy.copy(robot_node.getField("rotation").getSFRotation())


def supervisor_reset_to_home():
    global robot_node
    pos_field = robot_node.getField("translation")
    pos_field.setSFVec3f(start_translation)
    pos_field = robot_node.getField("rotation")
    pos_field.setSFRotation(start_rotation)
    supervisor.resetPhysics()
    print("Supervisor reset robot to start position")

def supervisor_get_relative_target_pose():
    '''
    Returns target position relative to the robot's current position.
    Do not call during your solution! Only during problem setup and for debugging!
    '''

    # Webots -X = Robot Y
    # Webots -Z = Robot X
    # Webots theta = Robot Theta
    robot_position = np.array(robot_node.getField("translation").getSFVec3f())
    target_position = np.array(target_node.getField("translation").getSFVec3f()) - robot_position
    theta = np.array(target_node.getField("rotation").getSFRotation()[3]) - np.array(robot_node.getField("rotation").getSFRotation()[3])
    target_pose = np.array([-target_position[2], -target_position[0], theta])
    # print("Target pose relative to robot: %s" % (str(target_pose)))
    return target_pose

def supervisor_get_robot_pose():
    """
    Returns robot position
    """
    robot_position = np.array(robot_node.getField("translation").getSFVec3f())
    robot_pose = np.array([robot_position[0], robot_position[2], robot_node.getField("rotation").getSFRotation()[3]])

    return robot_pose
    
