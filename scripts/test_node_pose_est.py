#!/usr/bin python

import sys
import rospy
import subprocess

from TiagoBears_grasp.grasp_class import Grasp
from TiagoBears_plan.task_class import Task

from TiagoBears_PoseEstimation.srv import PoseEstimation

from geometry_msgs.msg import Pose, Point, Quaternion

if __name__ == '__main__':
    try:
        ns = '/TiagoBears'
        rospy.init_node('grasp')
        
        subprocess.call("roslaunch TiagoBears_grasp load_config.launch", shell=True) #ee:=pal-gripper (default), ee:= robotiq-2f-85
        subprocess.call("roslaunch TiagoBears_plan load_config.launch", shell=True) #sim:=False (default), sim:=True

        task = Task()

        task.move_torso_to(0.06)
        rospy.sleep(0.5)
        cube_poses = []
        rospy.wait_for_service('PoseEstimation')
        pose_est_service = rospy.ServiceProxy('PoseEstimation', PoseEstimation)

        try:
            poseArray = pose_est_service("Hi :) This is a test...").poseArray

        except rospy.ServiceException as e: 
            print('Service call failed: %s'%e)
            sys.exit(1)

        def pose_in_origin(pose):
            pose = pose.pose.pose
            return pose.position.x == 0 and pose.position.y == 0 and pose.position.z == 0

        for pose in poseArray:
            if pose is not None and not pose_in_origin(pose):
                cube_poses.append(pose.pose.pose)

        task.move_torso_to(0.30)

        grasp_left = Grasp(is_left=True)
        grasp_right = Grasp(is_left=False)

        place_pos_left_start = rospy.get_param(ns + '/place_pos_left_start')
        place_pos_right_start = rospy.get_param(ns + '/place_pos_right_start')
        place_pose_left = Pose(position=Point(x=place_pos_left_start[0], 
                                              y=place_pos_left_start[1], 
                                              z=place_pos_left_start[2]), 
                                              orientation=Quaternion(w=1.0))
        place_pose_right = Pose(position=Point(x=place_pos_right_start[0], 
                                               y=place_pos_right_start[1], 
                                               z=place_pos_right_start[2]), 
                                               orientation=Quaternion(w=1.0))
        
        len_orig = len(cube_poses)

        while len(cube_poses) > len_orig - 5 and not rospy.is_shutdown():
            # choose closest cube
            min_dist_sq = 100 #m, should be impossible
            min_ind = -1
            for index, cube_pose in enumerate(cube_poses):
                # while cube.pose == None: rospy.sleep(0.1)
                # dist_sq = cube.pose.position.x**2 + cube.pose.position.y**2 + (cube.pose.position.z-1.0)**2
                dist_sq = (cube_pose.position.x-0.2)**2 + (abs(cube_pose.position.y) - 0.375)**2
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    min_ind = index
            
            print '=== Trying to pick cube {0} ==='.format(min_ind)
            cube_pose = cube_poses.pop(min_ind)
            try:
                task.remove_cube_collisions()
                # add_cubes_for_collision_but_not(cube.id, cubes, scene, robot.get_planning_frame())
                use_left = True if cube_pose.position.y > 0 else False

                pick_success = grasp_left.pick(cube_pose) if use_left else grasp_right.pick(cube_pose)
                
                if pick_success:
                    task.add_cubes_for_collision_except_poses(min_ind, cube_poses)
                    place_pose = place_pose_left if use_left else place_pose_right
                    print '=== Trying to place cube {0} ==='.format(min_ind)

                    grasp_left.place(place_pose) if use_left else grasp_right.place(place_pose)

                    # update pose
                    if use_left:
                        if place_pose.position.y > 0.06:
                            place_pose.position.y -= 0.06 # move 6 cm to the right, check for next line
                        else:
                            place_pose.position.x -= 0.06 # start next line
                            place_pose.position.y = 0.335
                    else:
                        if place_pose.position.y < -0.06:
                            place_pose.position.y += 0.06 # move 6 cm to the right, check for next line
                        else:
                            place_pose.position.x -= 0.06 # start next line
                            place_pose.position.y = -0.335

            except rospy.ROSInterruptException as e:
                    print('an exception has occured:')
                    print(e)

    except KeyboardInterrupt:
        sys.exit()
