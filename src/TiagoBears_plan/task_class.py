#!/usr/bin/env python

import rospy
import sys
from threading import Thread, Lock

import moveit_commander
import moveit_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

from TiagoBears_PoseEstimation.srv import PoseEstimation

class Task:
    """
    handles collision avoidance, setup tasks and torso movement

    About the change of structure in the torso value publishing: 
    https://answers.ros.org/question/9665/test-for-when-a-rospy-publisher-become-available/
    """

    def __init__(self, ns='/TiagoBears'):
        self.ns = ns

        # initialize move it for both arms
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
        self._robot = moveit_commander.RobotCommander(robot_description="robot_description")
        self.planning_frame = self._robot.get_planning_frame()

        # Instantiate a `PlanningSceneInterface`_ object; it's a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world:
        self._scene = moveit_commander.PlanningSceneInterface()

        # read out table dimensions from param server
        self._table_dim = rospy.get_param(self.ns + '/table_params')
        self._table_diff = rospy.get_param(self.ns + '/min_diff_to_table') # min dist to table for all grasps

        # add table as collision object
        self.add_table_collision()

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
        self.disp_traj_pub = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # rate used for all publishing matters
        self._rate = rospy.Rate(10) # 10 Hz

        # init to move torso to joint position 0.25
        self._torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
        self._torso_lock = Lock()
        self._torso_state = 0.25
        self._pub_thread = Thread(name='task_publisher', target=self._update_torso)
        self._pub_thread.start()
        self.move_torso_up()      

    ## Torso movement
    def move_torso_up(self):
        self.move_torso_to(0.30)

    def move_torso_down(self):
        self.move_torso_to(0.14)

    def move_torso_to(self, value):
        # value bounded to [0, 0.35] in m
        with self._torso_lock:
            self._torso_state = value
        
    def _update_torso(self):
        while not rospy.is_shutdown():
            with self._torso_lock:
                goal = [self._torso_state]
            msg = JointTrajectory(joint_names=['torso_lift_joint'], points=[JointTrajectoryPoint(positions=goal, time_from_start=rospy.Duration.from_sec(1))])
            self._torso_pub.publish(msg)
            self._rate.sleep()   

        sys.exit() 

    ## Cube estimation
    def get_cube_poses(self):
        # get the pose estimation from the pose estimation node, cube_poses sorted by increasing x
        self.move_torso_to(0.06)

        cube_poses = []
        pose_est_service = rospy.ServiceProxy('PoseEstimation', PoseEstimation)

        poseArray = None
        while poseArray is None:
            try:
                rospy.wait_for_service('PoseEstimation')
                poseArray = pose_est_service("Querying PoseEstimation service").poseArray

            except rospy.ServiceException as e: 
                print('Service call failed: %s'%e)
                rospy.sleep(1)

        self.move_torso_to(0.30)

        def pose_in_origin(pose):
            pose = pose.pose.pose
            return pose.position.x == 0 and pose.position.y == 0 and pose.position.z == 0

        for pose in poseArray:
            # check validity of pose
            if pose is not None and not pose_in_origin(pose): 
                # sort cube_poses in increasing order of x coordinate of pos
                for index, cube_pose in enumerate(cube_poses):
                    if cube_pose.position.x > pose.pose.pose.position.x:
                        cube_poses.insert(index, pose.pose.pose)
                        break

        return cube_poses

    ## Collision
    def add_cubes_for_collision_except(self, id_not_to_add, cubes):
        for cube in cubes:
            if cube.id is not id_not_to_add:
                ps = PoseStamped()
                ps.header.frame_id = self.planning_frame
                ps.pose = cube.pose
                self._scene.add_box('cube_{0}'.format(cube.id), ps, (0.045, 0.045, 0.045))

    def add_cubes_for_collision_except_poses(self, id_not_to_add, cube_poses):
        for index, cube_pose in enumerate(cube_poses):
            if index is not id_not_to_add:
                ps = PoseStamped()
                ps.header.frame_id = self.planning_frame
                ps.pose = cube_pose
                self._scene.add_box('cube_{0}'.format(index), ps, (0.045, 0.045, 0.045))

    def remove_cube_collisions(self):
        self._scene.remove_world_object()
        self.add_table_collision()

    def add_table_collision(self):
        # add table as collision object
        p = Pose(Point(x=0.5, y=0, z=0.21), Quaternion())
        ps = PoseStamped()
        ps.header.frame_id = self.planning_frame
        ps.pose = p
        
        self._scene.add_box('table', ps, (0.62, 0.78, 0.50))

    def add_table_collision_at(self, z_val):
        # add table as collision object
        p = Pose(Point(x=0.5, y=0, z=z_val), Quaternion())
        ps = PoseStamped()
        ps.header.frame_id = self.planning_frame
        ps.pose = p
        self._scene.add_box('table', ps, (0.62, 0.78, 0.50))