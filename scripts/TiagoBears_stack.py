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
        behavior = Behaviour_stack(ns)

        grasp_left = GraspWrapper('grasp_wrapper_left', is_left=True)
        grasp_right = GraspWrapper('grasp_wrapper_right', is_left=False)
        grasp_left.start()
        grasp_right.start()

        # get first cube pose estimation and take image to compare for later checks if cube is in gripper/picking has been successful
        task.init_image_grippers()
        
        cube_poses = task.get_cube_poses()
        next_cubes = behavior.get_next_cube_poses(cube_poses)
        
        last_place_pose_left = None
        last_place_pose_right = None

        go_on = True
        renew_pose_estimation = False
        while go_on and not rospy.is_shutdown():            
            # TODO: add freeing space

            ## Handling pick poses: filling in the next element, after the last one has been tried
            # the grasp_wrapper's pick pose is set to None the moment it will be tried
            if grasp_left.get_next_cube_pose() is None:
                grasp_left.set_next_cube_pose(next_cubes[0])
            if grasp_right.get_next_cube_pose() is None:
                grasp_right.set_next_cube_pose(next_cubes[1])

            ## Handling place poses: filling in the next element if needed
            # the grasp_wrapper's place pose is only set to None if the last placing was successful
            # calling behavior.get_next_place_pose will automatically update the place pose to the next one
            if grasp_left.get_next_place_pose() is None:
                task.add_cube_for_collision_at(last_place_pose_left)
                last_place_pose_left = behavior.get_next_place_pose_left()
                grasp_left.set_next_place_pose(last_place_pose_left)
                renew_pose_estimation = True # cube could have fallen down, renew pose estimation to cover this case

            if grasp_right.get_next_place_pose() is None:
                task.add_cube_for_collision_at(last_place_pose_right)
                last_place_pose_right = behavior.get_next_place_pose_right()
                grasp_right.set_next_place_pose(last_place_pose_right)
                renew_pose_estimation = True # cube could have fallen down, renew pose estimation to cover this case

            if renew_pose_estimation_left and :
                grasp_left.join()
                grasp_right.join()

                cube_poses = task.get_cube_poses()
                renew_pose_estimation = False

                grasp_left.continue_()
                grasp_right.continue_()

            next_cube = behavior.get_next_cube_pose(cube_poses)
            go_on = next_cube is not None or not (grasp_left.get_state() == GraspState.FREE and grasp_right.get_state() == GraspState.FREE)

            rospy.sleep(0.2)

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
