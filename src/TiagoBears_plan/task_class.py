#!/usr/bin/env python

import rospy
import sys
from threading import Thread, Lock

import moveit_commander
import moveit_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

from TiagoBears_PoseEstimation.srv import PoseEstimation, TableCornerDetection
from TiagoBears_ColorDetection.srv import InitEmpty
from TiagoBears_grasp.srv import Trigger

class Task:
    """
    handles collision avoidance, setup tasks and torso movement
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

        # add table as collision object
        self.remove_cube_collisions()

        self._cube_counter = 0  

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

    def init_image_grippers(self):
        init_img_req = rospy.ServiceProxy('/TiagoBears/init_empty_check', InitEmpty)
        
        success = None
        while success is None:
            try:
                rospy.wait_for_service('/TiagoBears/init_empty_check')
                success = init_img_req(True).res # bool should be irrelevant

            except rospy.ServiceException as e: 
                print('Service call failed: %s'%e)
                rospy.sleep(1)

    ## Cube estimation
    def get_cube_poses(self):
        """
         get the pose estimation from the pose estimation node, cube_poses sorted by increasing x
        """

        self._move_arms_to_start() # move arms out of field of view, avoid collision with table
        self.move_torso_to(0.06)
        rospy.sleep(2) # wait for torso to arrive

        poseArray = self._call_pose_estimation()

        self.move_torso_to(0.30)
        self._move_arms_to_watch() # move arms back to watch position

        def pose_in_origin(pose):
            pose = pose.pose.pose
            return pose.position.x == 0 and pose.position.y == 0 and pose.position.z == 0

        cube_poses = []
        for pose in poseArray:
            # check validity of pose
            if pose is not None and not pose_in_origin(pose): 
                # sort cube_poses in increasing order of x coordinate of pos
                pose = pose.pose.pose
                if len(cube_poses) == 0:
                    cube_poses.append(pose)
                else:
                    for index, cube_pose in enumerate(cube_poses):
                        if cube_pose is not None and cube_pose.position.x > pose.position.x:
                            cube_poses.insert(index, pose)
                            pose = None
                            break
                    if pose is not None: # has not yet been inserted
                        cube_poses.append(pose)

        return cube_poses

    def _call_pose_estimation(self):
        pose_est_service = rospy.ServiceProxy('/TiagoBears/PoseEstimation', PoseEstimation)
        poseArray = None
        while poseArray is None:
            try:
                rospy.wait_for_service('/TiagoBears/PoseEstimation')
                poseArray = pose_est_service("Querying PoseEstimation service").poseArray

            except rospy.ServiceException as e: 
                print('Service call failed: %s'%e)
                rospy.sleep(0.5)

        return poseArray

    ## Collision
    def fetch_table_dims(self):
        self._move_arms_to_start()

        table_detect_service = rospy.ServiceProxy('/TiagoBears/TableCornerDetection', TableCornerDetection)

        cornerPoints = None # will be a list of 4 points, [top_left, top_right, bot_left, bot_right]
        while cornerPoints is None:
            try:
                rospy.wait_for_service('/TiagoBears/TableCornerDetection')
                cornerPoints = table_detect_service('').cornerPointArray

            except rospy.ServiceException as e:
                print('Service call failed: %s'%e)
                rospy.sleep(0.5)

        # debug:
        print 'TableDetection returned:'
        for point in cornerPoints:
            print 'x: {0}, y: {1}, z: {2}'.format(point.x, point.y, point.z)

        x, y, z = self.add_table_collision_at(cornerPoints)
        self.set_table_dim([x, y, z])

        return cornerPoints

    def set_table_dim(self, table_dim):
        self._table_dim = table_dim

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

    def add_cube_for_collision_at(self, cube_pose):
        if cube_pose is None:
            return

        ps = PoseStamped()
        ps.header.frame_id = self.planning_frame
        ps.pose = cube_pose
        self._scene.add_box('cube_{0}'.format(self._cube_counter), ps, (0.045, 0.045, 0.045))

        self._cube_counter += 1

    def add_table_collision(self):
        # add table as collision object
        p = Pose(Point(x=0.6, y=0, z=self._table_dim[2]/2), Quaternion())
        
        ps = PoseStamped()
        ps.header.frame_id = self.planning_frame
        ps.pose = p
        
        self._scene.add_box('table', ps, tuple(self._table_dim))

    def add_table_collision_at(self, top_corner_points): 
        top_left, top_right, bot_left, bot_right = top_corner_points

        x_min = min(bot_left.x, bot_right.x) - 0.05
        x_max = max(top_left.x, top_right.x) + 0.05
        x = x_max - x_min
        
        y_min = min(top_right.y, bot_right.y) - 0.05
        y_max = max(top_left.y, bot_left.y) + 0.05
        y = y_max - y_min

        z = self._table_dim[2]
        self._scene.remove_world_object('table')
        
        # add table as collision object
        p = Pose(Point(x=0.6, y=0, z=z/2), Quaternion())
        ps = PoseStamped()
        ps.header.frame_id = self.planning_frame
        ps.pose = p
        self._scene.add_box('table', ps, (x, y, z))

        return x, y, z

    ## Arm movement
    def _move_arms_to_start(self):
        move_arm_service = rospy.ServiceProxy('/TiagoBears/go_to_start_left', Trigger)
        res = None
        while res is None:
            try:
                rospy.wait_for_service('/TiagoBears/go_to_start_left')
                res = move_arm_service(True).res

            except rospy.ServiceException as e: 
                print('Service call failed: %s'%e)
                rospy.sleep(0.2)

        move_arm_service = rospy.ServiceProxy('/TiagoBears/go_to_start_right', Trigger)
        res = None
        while res is None:
            try:
                rospy.wait_for_service('/TiagoBears/go_to_start_right')
                res = move_arm_service(True).res

            except rospy.ServiceException as e: 
                print('Service call failed: %s'%e)
                rospy.sleep(0.2)

    def _move_arms_to_watch(self):
        move_arm_service = rospy.ServiceProxy('/TiagoBears/go_to_watch_left', Trigger)
        res = None
        while res is None:
            try:
                rospy.wait_for_service('/TiagoBears/go_to_watch_left')
                res = move_arm_service(True).res

            except rospy.ServiceException as e: 
                print('Service call failed: %s'%e)
                rospy.sleep(0.2)

        move_arm_service = rospy.ServiceProxy('/TiagoBears/go_to_watch_right', Trigger)
        res = None
        while res is None:
            try:
                rospy.wait_for_service('/TiagoBears/go_to_watch_right')
                res = move_arm_service(True).res

            except rospy.ServiceException as e: 
                print('Service call failed: %s'%e)
                rospy.sleep(0.2)