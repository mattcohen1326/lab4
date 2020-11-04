"""csci3302_lab4 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import time
import numpy as np
from controller import Robot, Motor, DistanceSensor
import csci3302_lab4_supervisor


#state = "stationary" # Stay in place to test coordinate transform functions
state = "line_follower" # Drive along the course
USE_ODOMETRY = False # False for ground truth pose information, True for real odometry

###############################################################################################
## Start of Lab 4 Base Code, do not modify!
###############################################################################################

# Ground Sensor Measurements under this threshold are black
# measurements above this threshold can be considered white.
GROUND_SENSOR_THRESHOLD = 600 # Light intensity units
LIDAR_SENSOR_MAX_RANGE = 3. # Meters
LIDAR_ANGLE_BINS = 21 # 21 Bins to cover the angular range of the lidar, centered at 10
LIDAR_ANGLE_RANGE = 1.5708 # 90 degrees, 1.5708 radians

# These are your pose values that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0

#velocity reduction percent
MAX_VEL_REDUCTION = 0.2 # Run robot at 20% of max speed

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = .125 * MAX_VEL_REDUCTION # To be filled in with ePuck wheel speed in m/s

# Index into ground_sensors and ground_sensor_readings for each of the 3 onboard sensors.
LEFT_IDX = 0
CENTER_IDX = 1
RIGHT_IDX = 2
WHEEL_FORWARD = 1
WHEEL_STOPPED = 0
WHEEL_BACKWARD = -1

# create the Robot instance.
csci3302_lab4_supervisor.init_supervisor()
robot = csci3302_lab4_supervisor.supervisor

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
ground_sensor_readings = [0, 0, 0]
ground_sensors = [robot.getDistanceSensor('gs0'), robot.getDistanceSensor('gs1'), robot.getDistanceSensor('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# get and enable lidar 
lidar = robot.getLidar("LDS-01")
lidar.enable(SIM_TIMESTEP)
lidar.enablePointCloud()

#Initialize lidar motors
lidar_main_motor = robot.getMotor('LDS-01_main_motor')
lidar_secondary_motor = robot.getMotor('LDS-01_secondary_motor')
lidar_main_motor.setPosition(float('inf'))
lidar_secondary_motor.setPosition(float('inf'))
lidar_main_motor.setVelocity(30.0)
lidar_secondary_motor.setVelocity(60.0)


def update_odometry(left_wheel_direction, right_wheel_direction, time_elapsed):
    '''
    Given the amount of time passed and the direction each wheel was rotating,
    update the robot's pose information accordingly
    '''
    global pose_x, pose_y, pose_theta, EPUCK_MAX_WHEEL_SPEED, EPUCK_AXLE_DIAMETER
    # Update pose_theta
    pose_theta += (right_wheel_direction - left_wheel_direction) * time_elapsed * EPUCK_MAX_WHEEL_SPEED / EPUCK_AXLE_DIAMETER
    #Update pose_x
    pose_x += math.cos(pose_theta) * time_elapsed * EPUCK_MAX_WHEEL_SPEED * (left_wheel_direction + right_wheel_direction)/2.
    #Update pose_y
    pose_y += math.sin(pose_theta) * time_elapsed * EPUCK_MAX_WHEEL_SPEED * (left_wheel_direction + right_wheel_direction)/2.


###############################################################################################
## End of do not modify block.
###############################################################################################





########
## Part 1.1 - 1.2: Initialize your LIDAR-related data structures here
########
lidar_readings = []
offset_constant = LIDAR_ANGLE_RANGE/LIDAR_ANGLE_BINS
current_offset = 0
lidar_offsets = [0,1,2,3,4,5,6,7,8,9,0,11,12,13,14,15,16,17,18,19,20]
print(offset_constant)
for i in range (0,10):
    lidar_offsets[i] = (10-i)*-offset_constant
for i in range (11,21):
    lidar_offsets[i] = (i-10)*offset_constant
        
print(lidar_offsets)
########
## Part 3.1: Initialize your map data structure here
########
## 25x25 matrix each cell 4x4 cm

world_map = []


# YOUR CODE HERE





###
# Part 2.2



###
def convert_lidar_reading_to_world_coord(lidar_bin, lidar_distance):
    """
    @param lidar_bin: The beam index that provided this measurement
    @param lidar_distance: The distance measurement from the sensor for that beam
    @return world_point: List containing the corresponding (x,y) point in the world frame of reference
    """
    
    # YOUR CODE HERE
    B_q_x = math.cos(lidar_offsets[lidar_bin]) * lidar_distance
    B_q_y = math.sin(lidar_offsets[lidar_bin]) * lidar_distance
    world_point_x = (math.cos(pose_theta) * B_q_x) - (math.sin(pose_theta)*B_q_y) + pose_x
    world_point_y = (math.sin(pose_theta) * B_q_x) + (math.cos(pose_theta)*B_q_y) + pose_y
    
    return ([world_point_x, world_point_y])


###
# Part 3.2
###
def transform_world_coord_to_map_coord(world_coord):
    """
    @param world_coord: Tuple of (x,y) position in world coordinates
    @return grid_coord: Tuple of (i,j) coordinates corresponding to grid column and row in our map
    """
    if world_coord[0] >= 0 and world_coord[0] <= 1 and world_coord[1] >= 0 and world_coord[1] <= 1:
        pass
    else:
        return None
        
    return (world_coord[1]//.04, world_coord[0]//.04)
    # YOUR CODE HERE
    raise NotImplementedError


###
# Part 3.3
###
def transform_map_coord_world_coord(map_coord):
    """
    @param map_coord: Tuple of (i,j) coordinates corresponding to grid column and row in our map
    @return world_coord: Tuple of (x,y) position corresponding to the center of map_coord, in world coordinates
    """
    #12,12
    if map_coord[0] >= 0 and map_coord[0] <= 24 and map_coord[1] >= 0 and map_coord[1] <= 1 and map_coord[0]%1 == 0 and map_coord[1]%1 == 0:
        pass
    else:
        return None
        
    return(map_coord[1]*.04 +.02, map_coord[0]*.04 + .02)
    # YOUR CODE HERE
    raise NotImplementedError



###
# Part 3.5
###
def update_map(lidar_readings_array):
    """
    @param lidar_readings_array
    """
    # YOUR CODE HERE
    raise NotImplementedError


###
# Part 4.1
###
def display_map(m):
    """
    @param m: The world map matrix to visualize
    """
    # YOUR CODE HERE
    raise NotImplementedError


def main():
    global robot, ground_sensors, ground_sensor_readings, state
    global leftMotor, rightMotor, lidar, SIM_TIMESTEP
    global pose_x, pose_y, pose_theta
     
    ################################################################################
    # Do not modify:
    ################################################################################
    # Odometry variables
    last_odometry_update_time = None
    loop_closure_detection_time = 0
    
    # Variables to keep track of which direction each wheel is turning for odometry
    left_wheel_direction = 0
    right_wheel_direction = 0
    
    # Set starting pose of the robot by cheating (querying the simulator)
    robot_pose = csci3302_lab4_supervisor.supervisor_get_robot_pose()
    starting_pose = robot_pose[0], robot_pose[1], robot_pose[2]
    pose_x, pose_y, pose_theta = starting_pose
    
    # Burn a couple cycles waiting to let everything come online
    for i in range(10): robot.step(SIM_TIMESTEP)

    ################################################################################
    # Do not modify ^
    ################################################################################


    # Main Control Loop:
    while robot.step(SIM_TIMESTEP) != -1:     
        # Read ground sensor values
        for i, gs in enumerate(ground_sensors):
            ground_sensor_readings[i] = gs.getValue()
                   
        # If first time entering the loop, just take the current time as "last odometry update" time
        if last_odometry_update_time is None:
            last_odometry_update_time = robot.getTime()

        # Update Odometry
        time_elapsed = robot.getTime() - last_odometry_update_time
        last_odometry_update_time += time_elapsed
        update_odometry(left_wheel_direction, right_wheel_direction, time_elapsed)


        #################################################################################################
        # Control whether or not the robot is moving or stationary, and if using odometry or ground truth
        #################################################################################################
        if state == "line_follower":
            # Line Following Code
            if ground_sensor_readings[CENTER_IDX] < GROUND_SENSOR_THRESHOLD:
                leftMotor.setVelocity(leftMotor.getMaxVelocity()* MAX_VEL_REDUCTION)
                rightMotor.setVelocity(rightMotor.getMaxVelocity()* MAX_VEL_REDUCTION)
                left_wheel_direction, right_wheel_direction = WHEEL_FORWARD * MAX_VEL_REDUCTION, WHEEL_FORWARD * MAX_VEL_REDUCTION
            elif ground_sensor_readings[LEFT_IDX] < GROUND_SENSOR_THRESHOLD:
                leftMotor.setVelocity(-leftMotor.getMaxVelocity()* MAX_VEL_REDUCTION)
                rightMotor.setVelocity(rightMotor.getMaxVelocity()* MAX_VEL_REDUCTION)
                left_wheel_direction, right_wheel_direction = WHEEL_BACKWARD * MAX_VEL_REDUCTION, WHEEL_FORWARD* MAX_VEL_REDUCTION
            elif ground_sensor_readings[RIGHT_IDX] < GROUND_SENSOR_THRESHOLD:
                leftMotor.setVelocity(leftMotor.getMaxVelocity()* MAX_VEL_REDUCTION)
                rightMotor.setVelocity(-rightMotor.getMaxVelocity()* MAX_VEL_REDUCTION)
                left_wheel_direction, right_wheel_direction = WHEEL_FORWARD* MAX_VEL_REDUCTION, WHEEL_BACKWARD* MAX_VEL_REDUCTION
            else:
                leftMotor.setVelocity(-leftMotor.getMaxVelocity()* MAX_VEL_REDUCTION)
                rightMotor.setVelocity(rightMotor.getMaxVelocity()* MAX_VEL_REDUCTION)
                left_wheel_direction, right_wheel_direction = WHEEL_BACKWARD* MAX_VEL_REDUCTION, WHEEL_FORWARD* MAX_VEL_REDUCTION    
    
            # Loop Closure
            if False not in [ground_sensor_readings[i] < GROUND_SENSOR_THRESHOLD for i in range(3)]:
                loop_closure_detection_time += SIM_TIMESTEP / 1000.
                if loop_closure_detection_time > 0.1:
                    pose_x, pose_y, pose_theta = starting_pose
                    loop_closure_detection_time = 0
            else:
                loop_closure_detection_time = 0
        else:
            # Stationary State
            left_wheel_direction, right_wheel_direction = 0, 0
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)


        # Getting supervisor reading for robot. Can be replaced with odometry code 
        if USE_ODOMETRY is False:
            robot_pose = csci3302_lab4_supervisor.supervisor_get_robot_pose()
            pose_x, pose_y, pose_theta = robot_pose[0], robot_pose[1], robot_pose[2]

        #################################################################################################
        # Do not modify ^
        #################################################################################################
        
        
        # Debugging printouts that may be useful:        
        # print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta)) # Print our current pose
        #print("LIDAR: %s" % (str(lidar_readings))) # Print out the LIDAR reading in the terminal
        #print(lidar_offsets[10])
        ####
        # YOUR CODE HERE for Part 1.3, 3.4, and 4.1
        lidar_readings = lidar.getRangeImage()
        update_map()
        
        ####
        
    
if __name__ == "__main__":
    main()
