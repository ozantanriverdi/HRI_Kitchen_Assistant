#!/usr/bin/env python3

import rospy
import json
from pymycobot import MyCobot
from pymycobot import MyCobotSocket
from pymycobot import utils
import time
import sys
from pynput import keyboard
import moveit_commander
import moveit_msgs.msg
import shape_msgs.msg
from geometry_msgs.msg import Pose, Quaternion, Point
import tf.transformations
import math
import numpy as np
from std_msgs.msg import String
from pid_controller import PIDController

json_camera_path = '/home/ozan/catkin_ws/src/camera_input.json'
json_voice_path = '/home/ozan/catkin_ws/src/voice_input.json'

class Controller_Node():
    def __init__(self):
        rospy.init_node('move_moveit') # TODO: anonymous=True
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group_name = "arm_group"  # Adjust to your group
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.mc = MyCobot("/dev/ttyACM1", 115200)
        while self.mc is None:
            self.mc = MyCobot("/dev/ttyACM1", 115200)
            time.sleep(1)
        
        self.lastCommandIndex = -1
        self.controller = PIDController(kp=1.0, ki=1.0, kd=0.0)
        self.scale_value = 0.0
        self.dt = 0.1
        self.last_time = rospy.Time.now()
        self.desired_setpoint = None
        
        self.go_command = False
        self.tossing = 0
        self.robot_active = False
        self.stuck = False
        
        
        self.x_angle = 0
        self.y_angle = math.pi
        self.z_angle = math.pi * 3  / 4
        
        self.isSleep = True
        
        self.control_output = 0.0
        
        self.prior_angle = 0.0
        
        self.in_between_coordinates = [0.06, 0.15, 0.1]
        self.target_coordinates = [-0.13, 0.18, 0.1]
        self.pick_up_height = 0.03
        self.pick_up_height_above = 0.1
        
        self.original_coordinates = []
        
        
        rospy.Subscriber('scale', String, self.callback1)
        rospy.Subscriber('command', String, self.callback2)
        pub_feed = rospy.Publisher('feedback', String, queue_size=10)
        pub_stuck = rospy.Publisher('stuck', String, queue_size=10)
        rate = rospy.Rate(0.2)

        while not rospy.is_shutdown():
            
            a = self.robot_active == False
            temp = {"shouldListen": str(a)}
            temp = json.dumps(temp)
            pub_feed.publish(temp)
            rospy.loginfo('\nFeedback SENT: ' + str(temp))

            temp = {"stuck": str(self.stuck)}
            temp = json.dumps(temp)
            pub_stuck.publish(temp)
            rospy.loginfo('\stuck SENT: ' + str(temp))
            self.stuck = False
            
            
            rate.sleep()
    
    
    
    def callback1(self, msg):
        if self.tossing:
            current_time = rospy.Time.now()
            self.dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time
            
            self.value_adjust = float(msg.data)
            
            self.control_output = self.controller.update(setpoint=self.desired_setpoint, 
                                                    measured_value=self.value_adjust, 
                                                    dt=self.dt)
            if self.control_output < -160:
                print("Reached limit angle!!")

                self.control_output = -160
                
            elif self.control_output > 160:
                print("Reached limit angle!!")
                self.control_output = 160


            if self.value_adjust >= self.desired_setpoint:
                self.tossing = 0
                self.moveit_joints_tilt_back()
                time.sleep(1)
                self.move_moveit_constraint(self.in_between_coordinates[0], 
                                            self.in_between_coordinates[1], 
                                            self.in_between_coordinates[2]) # In-Between Steps
                time.sleep(1)
                self.move_moveit_constraint(self.original_coordinates[0], 
                                            self.original_coordinates[1], 
                                            self.pick_up_height_above) # Original - above
                time.sleep(1)
                self.move_moveit_constraint(self.original_coordinates[0], 
                                            self.original_coordinates[1], 
                                            self.pick_up_height) # Original
                time.sleep(1)
                self.mc.set_gripper_state(0, 100, 1)
                self.mc.set_gripper_state(0, 100, 1)
                time.sleep(1)
                self.move_moveit_constraint(self.original_coordinates[0], 
                                            self.original_coordinates[1], 
                                            self.pick_up_height_above) # Original - above
                time.sleep(1)
                self.move_moveit_constraint(self.in_between_coordinates[0], 
                                            self.in_between_coordinates[1], 
                                            self.in_between_coordinates[2]) # In-Between Steps
                self.robot_active = False
            else:
                self.moveit_joints_tilt(self.control_output)
        

    def callback2(self, data):
        self.lastCommandIndex
        data = json.loads(data.data)
        rospy.loginfo("\ncommand RECEIVED = " + str(data))
        if 'index' in data.keys():
            if self.lastCommandIndex != data['index']:
                self.lastCommandIndex = data['index']
                self.original_coordinates = data['coords']
                self.desired_setpoint = data['amount']
                self.robot_active = True
                self.go_command = True

        if self.go_command:
            self.move_moveit_loose(self.in_between_coordinates[0], 
                                            self.in_between_coordinates[1], 
                                            self.in_between_coordinates[2]) # In-Between Steps
            time.sleep(1)
            self.move_moveit_loose(self.original_coordinates[0], 
                                            self.original_coordinates[1], 
                                            self.pick_up_height_above) # Original - above
            time.sleep(1)
            self.mc.set_gripper_state(0, 100, 1)
            self.mc.set_gripper_state(0, 100, 1)
            time.sleep(1)
            self.move_moveit_constraint(self.original_coordinates[0], 
                                            self.original_coordinates[1], 
                                            self.pick_up_height) # Original
            time.sleep(1)
            self.mc.set_gripper_state(1, 100, 1)
            self.mc.set_gripper_state(1, 100, 1)
            time.sleep(1)
            self.move_moveit_constraint(self.original_coordinates[0], 
                                            self.original_coordinates[1], 
                                            self.pick_up_height_above) # Original - above
            time.sleep(1)
            self.move_moveit_constraint(self.in_between_coordinates[0], 
                                            self.in_between_coordinates[1], 
                                            self.in_between_coordinates[2]) # In-Between Steps
            time.sleep(1)
            self.move_moveit_constraint(self.target_coordinates[0], 
                                        self.target_coordinates[1], 
                                        self.target_coordinates[2]) # Target
            self.tossing = 1
            self.go_command = 0
        #print("You can deposit!!")
    

    def move_moveit_loose(self, x, y, z):
        
        joint_names = self.robot.get_active_joint_names()
        link_names = self.robot.get_active_joint_names()
        print("Joint Names:", joint_names)
        print("Link Names:", link_names)
        current = self.move_group.get_current_pose()
        print("Current Pose:\n", current.pose.position)
        
        rotations = [self.x_angle, self.y_angle, self.z_angle]
        target_orientation_numpy = tf.transformations.quaternion_from_euler(*rotations)
        #print("Quaternion_numpy: ", target_orientation_numpy, type(target_orientation_numpy))
        target_orientation_msg = Quaternion(*target_orientation_numpy)
        #print("Quaternion: ", target_orientation_msg, type(target_orientation_msg))
        
        target_pose = Pose()
        target_pose.orientation = target_orientation_msg
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        print("Target Pose:\n", target_pose.position)
        self.move_group.set_pose_target(target_pose)
        
        print("Before Execution")
        #success = move_group.go(wait=True)
        for i in range(10):
            self.stuck = True
            success = self.move_group.go(wait=True)
            if success:
                print(success)
                break
            print(success)
            self.robot_active = True
            

        # if success == isSleep:
        #      time.sleep(5)
                
        self.move_group.stop()  # Ensure there's no residual movement
        self.move_group.clear_pose_targets()
        

    def move_moveit_constraint(self, x, y, z):
        
        joint_names = self.robot.get_active_joint_names()
        link_names = self.robot.get_active_joint_names()
        print("Joint Names:", joint_names)
        print("Link Names:", link_names)
        current = self.move_group.get_current_pose()
        print("Current Pose:\n", current.pose.position)
        
        rotations = [self.x_angle, self.y_angle, self.z_angle]
        target_orientation_numpy = tf.transformations.quaternion_from_euler(*rotations)
        #print("Quaternion_numpy: ", target_orientation_numpy, type(target_orientation_numpy))
        target_orientation_msg = Quaternion(*target_orientation_numpy)
        #print("Quaternion: ", target_orientation_msg, type(target_orientation_msg))
        
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        #orientation_constraint.header.frame_id = move_group.get_planning_frame()
        orientation_constraint.link_name = 'joint6output_to_joint6'
        orientation_constraint.orientation = target_orientation_msg
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.5
        #orientation_constraint.weight = 1.0
        #print("Orientation Constraint: ", orientation_constraint)
        
        constraints = moveit_msgs.msg.Constraints()
        constraints.orientation_constraints.append(orientation_constraint)
        self.move_group.set_path_constraints(constraints)
        
        target_pose = Pose()
        target_pose.orientation = target_orientation_msg
        self.move_group.set_goal_position_tolerance(0.005)
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        print("Target Pose:\n", target_pose.position)
        self.move_group.set_pose_target(target_pose)
        
        print("Before Execution")
        #success = move_group.go(wait=True)
        for i in range(10):
            success = self.move_group.go(wait=True)
            if success:
                print(success)
                break
            print(success)
            self.robot_active = True
        # if success == isSleep:
        #     time.sleep(5)
        # if success == False:
        #     for i in range(100):
        #         target_pose.position.x -= 0.002
        #         move_group.set_pose_target(target_pose)
        #         success = move_group.go(wait=True)
        #         print(success)
                
        self.move_group.stop()  # Ensure there's no residual movement
        self.move_group.clear_path_constraints()  # Clear constraints for future operations
        self.move_group.clear_pose_targets()
    

    def moveit_joints_tilt(self, control_output):
        tau = math.pi * 2
        joint_goal = self.move_group.get_current_joint_values()
        self.prior_angle = joint_goal[4]
        joint_goal[4] = (tau / 360) * control_output
        self.move_group.go(joint_goal, wait=True)

    def moveit_joints_tilt_back(self):
        tau = math.pi * 2
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[4] = self.prior_angle
        self.move_group.go(joint_goal, wait=True)























# def move_moveit_constraint_straight_z(x, y, z, x_angle, y_angle, z_angle, isSleep=True):
    
#     joint_names = robot.get_active_joint_names()
#     link_names = robot.get_active_joint_names()
#     print("Joint Names:", joint_names)
#     print("Link Names:", link_names)
#     current = move_group.get_current_pose()
#     print("Current Pose:\n", current.pose.position)
    
#     rotations = [x_angle, y_angle, z_angle]
#     target_orientation_numpy = tf.transformations.quaternion_from_euler(*rotations)
#     #print("Quaternion_numpy: ", target_orientation_numpy, type(target_orientation_numpy))
#     target_orientation_msg = Quaternion(*target_orientation_numpy)
#     #print("Quaternion: ", target_orientation_msg, type(target_orientation_msg))
    
#     #orientation_constraint = moveit_msgs.msg.OrientationConstraint()
#     # TODO: JointConstraint
#     joint_constraint = moveit_msgs.msg.JointConstraint()
#     joint_constraint.joint_name = 'joint2_to_joint1'
#     #joint_constraint.header.frame_id = move_group.get_planning_frame()
#     joint_constraint.position = z
#     joint_constraint.tolerance_above = 0.1  # Set tolerance for values above the desired position
#     joint_constraint.tolerance_below = 0.1  # Set tolerance for values below the desired position
#     joint_constraint.weight = 1.0
    
#     constraints = moveit_msgs.msg.Constraints()
#     constraints.joint_constraints.append(joint_constraint)
#     move_group.set_path_constraints(constraints)
    
#     target_pose = Pose()
#     target_pose.orientation = target_orientation_msg
#     #move_group.set_goal_position_tolerance(0.005)
#     target_pose.position.x = current.pose.position.x
#     target_pose.position.y = current.pose.position.y
#     target_pose.position.z = z
#     print("Target Pose:\n", target_pose.position)
#     move_group.set_pose_target(target_pose)
    
#     print("Before Execution")
#     #success = move_group.go(wait=True)
#     for i in range(10):
#         success = move_group.go(wait=True)
#         if success:
#             print(success)
#             break
#         print(success)
#     # if success == isSleep:
#     #     time.sleep(5)
#     # if success == False:
#     #     for i in range(100):
#     #         target_pose.position.x -= 0.002
#     #         move_group.set_pose_target(target_pose)
#     #         success = move_group.go(wait=True)
#     #         print(success)
            
#     move_group.stop()  # Ensure there's no residual movement
#     move_group.clear_path_constraints()  # Clear constraints for future operations
#     move_group.clear_pose_targets()
    
# def move_moveit_constraint_straight_z_gemini(x, y, z, x_angle, y_angle, z_angle, isSleep=True):
#     joint_names = robot.get_active_joint_names()
#     current = move_group.get_current_pose()

#     rotations = [x_angle, y_angle, z_angle]
#     target_orientation_numpy = tf.transformations.quaternion_from_euler(*rotations)
#     target_orientation_msg = Quaternion(*target_orientation_numpy)
    
#     # Joint constraint for straight-down movement:
#     joint_constraint = moveit_msgs.msg.JointConstraint()
#     # Specify the joint responsible for Z-axis motion (replace if different):
#     joint_constraint.joint_name = 'joint2_to_joint1'  # Replace with correct joint name
#     joint_constraint.position = current.pose.position.z  # Lock Z-axis position

#     # Optional: Add limits for other joints if needed
#     for joint_name in joint_names:
#         if joint_name != 'joint2_to_joint1':
#             joint_constraint.tolerance_above = 0.05  # Example tolerances
#             joint_constraint.tolerance_below = 0.05

#     constraints = moveit_msgs.msg.Constraints()
#     constraints.joint_constraints.append(joint_constraint)
#     move_group.set_path_constraints(constraints)

#     # Target pose only for orientation (X and Y remain unchanged):
#     target_pose = Pose()
#     target_pose.orientation = target_orientation_msg

#     # Set target pose and execute movement:
#     move_group.set_pose_target(target_pose)
#     print("Before Execution")
#     #success = move_group.go(wait=True)
#     for i in range(10):
#         success = move_group.go(wait=True)
#         if success:
#             print(success)
#             break
#         print(success)
#     move_group.stop()  # Ensure there's no residual movement
#     move_group.clear_pose_targets()
#     print("Planning and execution successful:", success)


# def move_moveit_constraint_straight_z_gemini_2(x, y, z, x_angle, y_angle, z_angle, isSleep=True):
#     joint_names = robot.get_active_joint_names()
#     link_names = robot.get_active_joint_names()

#     current = move_group.get_current_pose()

#     rotations = [x_angle, y_angle, z_angle]
#     target_orientation = tf.transformations.quaternion_from_euler(*rotations)
#     target_orientation_msg = Quaternion(*target_orientation)

#     # Position constraint for limited movement (less precise than joint constraint)
#     position_constraint = moveit_msgs.msg.PositionConstraint()
#     position_constraint.header.frame_id = move_group.get_planning_frame()
#     position_constraint.link_name = move_group.get_end_effector_link()  # Target link at end-effector

#     # Set target position with fixed X and Y, variable Z
#     target_position = Point()
#     target_position.x = current.pose.position.x  # Maintain current X
#     target_position.y = current.pose.position.y  # Maintain current Y
#     target_position.z = z  # Target Z position

#     volume = moveit_msgs.msg.BoundingVolume()
#     box_size = shape_msgs.msg.SolidPrimitive()
#     box_size.type = shape_msgs.msg.SolidPrimitive.BOX
#     box_size.dimensions = [0.05, 0.05, 0.1]  # Tolerances (X, Y, Z)
#     volume.primitives.append(box_size)
#     volume.primitive_poses.append(Pose(position=target_position))
    
#     position_constraint.constraint_region = volume

#     constraints = moveit_msgs.msg.Constraints()
#     constraints.position_constraints.append(position_constraint)
#     move_group.set_path_constraints(constraints)

#     # Target pose only for orientation:
#     target_pose = Pose()
#     target_pose.orientation = target_orientation_msg

#     # Set target pose and execute movement:
#     move_group.set_pose_target(target_pose)
#     print("Before Execution")
#     #success = move_group.go(wait=True)
#     for i in range(10):
#         success = move_group.go(wait=True)
#         if success:
#             print(success)
#             break
#         print(success)

#     print("Planning and execution successful:", success)

#     move_group.stop()  # Ensure there's no residual movement
#     move_group.clear_pose_targets()







# def move_moveit_loose(x, y, z, x_angle, y_angle, z_angle, isSleep=True):
    
#     joint_names = robot.get_active_joint_names()
#     link_names = robot.get_active_joint_names()
#     print("Joint Names:", joint_names)
#     print("Link Names:", link_names)
#     current = move_group.get_current_pose()
#     print("Current Pose:\n", current.pose.position)
    
#     rotations = [x_angle, y_angle, z_angle]
#     target_orientation_numpy = tf.transformations.quaternion_from_euler(*rotations)
#     #print("Quaternion_numpy: ", target_orientation_numpy, type(target_orientation_numpy))
#     target_orientation_msg = Quaternion(*target_orientation_numpy)
#     #print("Quaternion: ", target_orientation_msg, type(target_orientation_msg))
    
#     target_pose = Pose()
#     target_pose.orientation = target_orientation_msg
#     target_pose.position.x = x
#     target_pose.position.y = y
#     target_pose.position.z = z
#     print("Target Pose:\n", target_pose.position)
#     move_group.set_pose_target(target_pose)
    
#     print("Before Execution")
#     #success = move_group.go(wait=True)
#     for i in range(10):
#         success = move_group.go(wait=True)
#         if success:
#             print(success)
#             break
#         print(success)

        
#     # if success == isSleep:
#     #      time.sleep(5)
            
#     move_group.stop()  # Ensure there's no residual movement
#     move_group.clear_pose_targets()
    
    
    
# def moveit_joints(control_output):
#     tau = math.pi * 2
#     joint_goal = move_group.get_current_joint_values()
#     joint_goal[4] = (tau / 360) * control_output
#     move_group.go(joint_goal, wait=True)
    
    
if __name__ == '__main__':
    
    controller_node = Controller_Node()
    
    
    
    
    
    
    
    
    
    
    # rospy.init_node('move_moveit')
    # moveit_commander.roscpp_initialize(sys.argv)
    # robot = moveit_commander.RobotCommander()
    # group_name = "arm_group"  # Adjust to your group
    # move_group = moveit_commander.MoveGroupCommander(group_name)
    # port = MyCobot("/dev/ttyACM1", 115200)
    # while port is None:
    #     mc = MyCobot("/dev/ttyACM1", 115200)
    #     time.sleep(1)
    
    
    #mc.set_gripper_value(100, 20)
    # joint_values = move_group.get_current_joint_values()
    # print(joint_values, type(joint_values))
    
    # moveit_joints()
    # print(joint_values, type(joint_values))

    
    

    
    
    #Positions for above salt
    # x = 0.2
    # y = 0.12
    # z = 0.1
    # x_angle=0
    # y_angle=math.pi
    # z_angle= 3 * math.pi/4
    # Move it to correct position
    #move_moveit(x, y, z, x_angle, y_angle, z_angle) # This moves the robot above the salt
    #print("Angles " , mc.get_angles())
    # x_angle_tossing = x_angle - 0.1 # TODO: change the correct angle to the correct value so the gripper tilts like you want it to
    # y_angle_tossing = y_angle
    # z_angle_tossing = z_angle - 0.1
    
    
    # x = 0.06
    # y = 0.15
    # z = 0.1
    # move_moveit_loose(x, y, z, x_angle, y_angle, z_angle) # In-Between Steps
    # time.sleep(1)
    # x = 0.2
    # y = 0.12
    # move_moveit_loose(x, y, z, x_angle, y_angle, z_angle) # Original - high
    # time.sleep(1)
    # mc.set_gripper_state(0, 100, 1)
    # mc.set_gripper_state(0, 100, 1)
    # time.sleep(1)
    # z = 0.03
    # # TODO: JointConstraint
    # move_moveit_constraint_straight_z_gemini_2(x, y, z, x_angle, y_angle, z_angle) # Original
    # time.sleep(1)
    # mc.set_gripper_state(1, 100, 1)
    # mc.set_gripper_state(1, 100, 1)
    # time.sleep(1)
    # z = 0.1
    # move_moveit_constraint_straight_z(x, y, z, x_angle, y_angle, z_angle) # Original - high
    # time.sleep(1)
    # x = 0.0
    # y = 0.15
    # move_moveit_constraint(x, y, z, x_angle, y_angle, z_angle) # In-Between Steps
    # time.sleep(1)
    # x = -0.13
    # y = 0.18
    # move_moveit_constraint(x, y, z, x_angle, y_angle, z_angle) # Target
    # time.sleep(1)
    # # TODO: Here comes the tilting motion
    # x = 0.0
    # y = 0.15
    # move_moveit_constraint(x, y, z, x_angle, y_angle, z_angle) # In-Between Steps
    # time.sleep(1)
    # x = 0.2
    # y = 0.12
    # move_moveit_constraint(x, y, z, x_angle, y_angle, z_angle) # Original - high
    # time.sleep(1)
    # z = 0.03
    # move_moveit_constraint_straight_z(x, y, z, x_angle, y_angle, z_angle) # Original
    # time.sleep(1)
    # mc.set_gripper_state(0, 100, 1)
    # mc.set_gripper_state(0, 100, 1)
    # time.sleep(1)
    # z = 0.1
    # move_moveit_constraint(x, y, z, x_angle, y_angle, z_angle) # Original - high
    # time.sleep(1)
    # x = 0.0
    # y = 0.15
    # z = 0.1
    # move_moveit_loose(x, y, z, x_angle, y_angle, z_angle) # In-Between Steps
    
    
    
    
    # TODO: Scale position
    # x = -0.22
    # y = 0.16
    # z = 0.1    
    
    
    
    
    
    
    
    # #move_moveit_loose(x, y, z, x_angle, y_angle, z_angle)
    # x = 0.15
    # y = -0.03
    # z = 0.1
    # # move_moveit_loose(x, y, z, x_angle, y_angle, z_angle)
    # print("should work till here")
    # move_moveit_constraint(x, y, z, x_angle, y_angle, z_angle)
    # # move_group.set_goal_orientation_tolerance()
    # x = 0.15
    # y = 0.15
    # z = 0.1
    # move_moveit_constraint(x, y, z, x_angle, y_angle, z_angle)
    # #move_moveit_loose(x, y, z, x_angle, y_angle, z_angle)
    
    # #print("rotating")
    # #move_moveit(x, y, z, x_angle_tossing, y_angle_tossing, z_angle_tossing)
    # # if tossing == "doesnt toss anymore":
    # #     move_moveit(x, y, z, x_angle, y_angle, z_angle)
    # # mc.send_angles([15, -85, -20, 0, 0, 45], 20)
    # #time.sleep(2)

    
    
    # TODO:
    # 1- ompl_planning.yaml -> add a line under RRT so it uses the shortest path
    # 2- target_pose.position.z = 0.03