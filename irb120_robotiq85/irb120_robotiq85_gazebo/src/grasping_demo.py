#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = 'Li Hao'
__date__ = '2020.03.30'
__copyright__ = "Copyright 2020, PI"


import sys
import math

import rospy
import moveit_python
import moveit_commander
import geometry_msgs.msg
import trajectory_msgs.msg
from tf import transformations
from moveit_msgs.msg import Grasp, PlaceLocation


PI = math.pi


def addBox(scene, name, frame, pose_xyz, size_xyz):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = frame
    box_pose.pose.position.x = pose_xyz[0]
    box_pose.pose.position.y = pose_xyz[1]
    box_pose.pose.position.z = pose_xyz[2]
    scene.add_box(name, box_pose, size_xyz)


def initEnvironment(scene):
    addBox(scene, "table", "/world", [0.5, 0, 0.2], [0.2, 0.4, 0.4])
    addBox(scene, "object1", "/world", [0.5, 0, 0.5], [0.02, 0.02, 0.2])
    addBox(scene, "object2", "/world", [0.5, -0.1, 0.45], [0.02, 0.02, 0.1])


def openGripper(posture):
    posture.joint_names.append("gripper_finger1_joint")
    posture.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    posture.points[0].positions.append(0)
    posture.points[0].time_from_start = rospy.Duration.from_sec(0.5)


def closeGripper(posture):
    posture.joint_names.append("gripper_finger1_joint")
    posture.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    posture.points[0].positions.append(0.5)
    posture.points[0].time_from_start = rospy.Duration.from_sec(0.5)


def pickup(pickPlaceInterface, frame, graspPose_xyzrpy, objectName, supportName):
    moveit_grasp = Grasp()
    moveit_grasp.grasp_pose.header.frame_id = frame
    moveit_grasp.grasp_pose.pose.orientation = geometry_msgs.msg.Quaternion(
        *transformations.quaternion_from_euler(*graspPose_xyzrpy[3:])
    )
    moveit_grasp.grasp_pose.pose.position.x = graspPose_xyzrpy[0]
    moveit_grasp.grasp_pose.pose.position.y = graspPose_xyzrpy[1]
    moveit_grasp.grasp_pose.pose.position.z = graspPose_xyzrpy[2]

    moveit_grasp.pre_grasp_approach.direction.header.frame_id = frame
    moveit_grasp.pre_grasp_approach.direction.vector.x = 1.0
    moveit_grasp.pre_grasp_approach.min_distance = 0.04
    moveit_grasp.pre_grasp_approach.desired_distance = 0.08

    moveit_grasp.post_grasp_retreat.direction.header.frame_id = frame
    moveit_grasp.post_grasp_retreat.direction.vector.z = 1.0
    moveit_grasp.post_grasp_retreat.min_distance = 0.1
    moveit_grasp.post_grasp_retreat.desired_distance = 0.25

    openGripper(moveit_grasp.pre_grasp_posture)
    closeGripper(moveit_grasp.grasp_posture)

    for i in range(10):
        res = pickPlaceInterface.pickup(objectName, [moveit_grasp,], support_name=supportName)
        # print("{}: {}".format(i, res.error_code.val))
        if res.error_code.val == 1:
            return True
    return False


def place(pickPlaceInterface, frame, placePose_xyzrpy, objectName, supportName):
    moveit_place = PlaceLocation()
    moveit_place.place_pose.header.frame_id = frame
    moveit_place.place_pose.pose.orientation = geometry_msgs.msg.Quaternion(
        *transformations.quaternion_from_euler(*placePose_xyzrpy[3:])
    )
    moveit_place.place_pose.pose.position.x = placePose_xyzrpy[0]
    moveit_place.place_pose.pose.position.y = placePose_xyzrpy[1]
    moveit_place.place_pose.pose.position.z = placePose_xyzrpy[2]

    moveit_place.pre_place_approach.direction.header.frame_id = frame
    moveit_place.pre_place_approach.direction.vector.z = -1.0
    moveit_place.pre_place_approach.min_distance = 0.095
    moveit_place.pre_place_approach.desired_distance = 0.115

    moveit_place.post_place_retreat.direction.header.frame_id = frame
    moveit_place.post_place_retreat.direction.vector.x = -1.0
    moveit_place.post_place_retreat.min_distance = 0.095
    moveit_place.post_place_retreat.desired_distance = 0.115

    openGripper(moveit_place.post_place_posture)

    for i in range(10):
        res = pickPlaceInterface.place(objectName, [moveit_place, ], support_name=supportName, goal_is_eef=True)
        # print("{}: {}".format(i, res.error_code.val))
        if res.error_code.val == 1:
            return True
    return False


def main():
    # Init stuff
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moving_irb120_robot', anonymous=True)
    # robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    # arm_group = moveit_commander.MoveGroupCommander("irb_120")
    # hand_group = moveit_commander.MoveGroupCommander("robotiq_85")
    # moveit_group = moveit_python.MoveGroupInterface("irb_120", "/world")
    pick_place_interface = moveit_python.PickPlaceInterface("irb_120", "robotiq_85")

    # # Publish trajectory in RViz
    # display_trajectory_publisher = rospy.Publisher(
    #     '/move_group/display_planned_path',
    #     moveit_msgs.msg.DisplayTrajectory,
    #     queue_size=20
    # )

    initEnvironment(scene)
    rospy.sleep(1)

    print("picking up object1...")
    pick_success_1 = pickup(pick_place_interface, "/world", [0.415, 0, 0.5, 0, 0, 0], "object1", "table")
    if not pick_success_1:
        print("pickup failed, exit")
        exit(-1)
    print("placing object1...")
    place_success_1 = place(pick_place_interface, "/world", [0.415, 0.1, 0.5, 0, 0, 0], "object1", "table")
    if not place_success_1:
        print("place failed, exit")
        exit(-1)

    print("picking up object2...")
    pick_success_2 = pickup(pick_place_interface, "/world", [0.415, -0.1, 0.45, 0, 0, 0], "object2", "table")
    if not pick_success_2:
        print("pickup failed, exit")
        exit(-1)
    print("placing object2...")
    place_success_2 = place(pick_place_interface, "/world", [0.415, 0.0, 0.45, 0, 0, 0], "object2", "table")
    if not place_success_2:
        print("place failed, exit")
        exit(-1)


if __name__ == '__main__':
    main()
