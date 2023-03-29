#!/usr/bin/env python

#!/usr/bin/env python

import sys
import rospy
import copy

from TiagoBears_grasp.grasp_class import Grasp
from TiagoBears_plan.task_class import Task
from TiagoBears_plan.behaviour_class import Behaviour_stack
from TiagoBears_plan.grasp_wrapper_class import GraspState, GraspWrapper

from geometry_msgs.msg import Pose, Point, Quaternion

if __name__ == '__main__':
    try:
        ns = '/TiagoBears'
        rospy.init_node('TiagoBears_stack')

        task = Task(ns) # already adds table as colllision object
        table_corners = task.fetch_table_dims() # from pose_estimation, also updates table collision object
        behavior = Behaviour_stack(ns, table_corners=table_corners)

        grasp_left = GraspWrapper('grasp_wrapper_left', is_left=True)
        grasp_right = GraspWrapper('grasp_wrapper_right', is_left=False)
        grasp_left.start()
        grasp_right.start()
        
        cube_poses = task.get_cube_poses()
        next_cubes = list(behavior.get_next_cube_poses(cube_poses)) # [next_cube_left, next_cube_right] as Poses of geometry_msgs
        task.init_image_grippers() # get first cube pose estimation and take image to compare for later checks if cube is in gripper/picking has been successful
        
        last_place_pose_left = None
        last_place_pose_right = None

        def correct_pose(cube_pose):
                """moving the cube_pose in y direction due to point cloud distortion that tilts table (see rviz)"""
                if cube_pose is None: return
                y = cube_pose.position.y
                # cube_pose.position.y += np.sign(y)*(0.045 * np.abs(y - (-0.15)) / 0.16)
                cube_pose.position.y += 0.04
                cube_pose.position.x += 0.01

                return cube_pose

        go_on = True
        task.add_table_collision()
        while go_on and not rospy.is_shutdown():            
            # TODO: add freeing space

            ## Handling pick poses: filling in the next element, after the last one has been tried
            # the grasp_wrapper's pick pose is set to None the moment it will be tried
            if grasp_left.get_next_cube_pose() is None:
                next_cubes[0] = correct_pose(next_cubes[0])
                grasp_left.set_next_cube_pose(next_cubes[0])
            if grasp_right.get_next_cube_pose() is None:
                next_cubes[1] = correct_pose(next_cubes[1])
                grasp_right.set_next_cube_pose(next_cubes[1])

            ## Handling place poses: filling in the next element if needed
            # the grasp_wrapper's place pose is only set to None if the last placing was successful
            # calling behavior.get_next_place_pose will automatically update the place pose to the next one
            if grasp_left.get_next_place_pose() is None:
                task.add_cube_for_collision_at(last_place_pose_left)
                last_place_pose_left = behavior.get_next_place_pose_left()
                last_place_pose_left = correct_pose(last_place_pose_left)
                grasp_left.set_next_place_pose(last_place_pose_left)

            if grasp_right.get_next_place_pose() is None:
                task.add_cube_for_collision_at(last_place_pose_right)
                last_place_pose_right = behavior.get_next_place_pose_right()
                last_place_pose_right = correct_pose(last_place_pose_right)
                grasp_right.set_next_place_pose(last_place_pose_right)

            ## renew pose estimation: cube could have fallen down, renew pose estimation to cover this case
            if grasp_left.get_state() in [GraspState.IS_PICKING, GraspState.IS_PLACING] or \
               grasp_right.get_state() in [GraspState.IS_PICKING, GraspState.IS_PLACING]:
                
                grasp_left.join()
                grasp_right.join()

                cube_poses = task.get_cube_poses()

                grasp_left.continue_()
                grasp_right.continue_()

            next_cubes = list(behavior.get_next_cube_poses(cube_poses))
            rospy.sleep(0.2)

            go_on = next_cubes is not (None, None) or not (grasp_left.get_state() == GraspState.FREE and grasp_right.get_state() == GraspState.FREE)


        print('All detected cubes have been stacked, exiting...')
        grasp_left.stop()
        grasp_right.stop()
        sys.exit()

    except rospy.ROSInterruptException as e:
        print('an exception has occured:')
        print(e)

    except KeyboardInterrupt:
        grasp_left.stop()
        grasp_right.stop()
        sys.exit()
