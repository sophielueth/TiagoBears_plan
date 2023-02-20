#!/usr/bin/env python

import rospy
import sys
from threading import Thread, Lock

import moveit_commander
import moveit_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

class Task:
    """
    handles collision avoidance, setup tasks and torso movement

    About the change of structure in the torso value publishing: 
    https://answers.ros.org/question/9665/test-for-when-a-rospy-publisher-become-available/
    """

    def __init__(self):
        # initialize move it for both arms
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
        self._robot = moveit_commander.RobotCommander(robot_description="robot_description")
        self.planning_frame = self._robot.get_planning_frame()

        # Instantiate a `PlanningSceneInterface`_ object; it's a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world:
        self._scene = moveit_commander.PlanningSceneInterface()

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

    ## Collision
    def add_cubes_for_collision_except(self, id_not_to_add, cubes):
        for cube in cubes:
            if cube.id is not id_not_to_add:
                ps = PoseStamped()
                ps.header.frame_id = self.planning_frame
                ps.pose = cube.pose
                self._scene.add_box('cube_{0}'.format(cube.id), ps, (0.045, 0.045, 0.045))

    def remove_cube_collisions(self):
        self._scene.remove_world_object()
        self.add_table_collision()

    def add_table_collision(self):
        # add table as collision object
        p = Pose(Point(x=0.5, y=0, z=0), Quaternion())
        ps = PoseStamped()
        ps.header.frame_id = self.planning_frame
        ps.pose = p
        self._scene.add_box('table', ps, (0.6, 0.75, 0.5))
        