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

        task = Task(ns)
        behavior = Behaviour_stack(ns)

        grasp_left = GraspWrapper('grasp_wrapper_left', is_left=True)
        grasp_right = GraspWrapper('grasp_wrapper_right', is_left=False)
        grasp_left.start()
        grasp_right.start()

        # get first cube pose estimation and take image to compare for later checks if cube is in gripper/picking has been successful
        cube_poses = task.get_cube_poses()
        next_cubes = behavior.get_next_cube_poses(cube_poses)
        task.init_image_grippers()

        go_on = True
        while go_on and not rospy.is_shutdown():            
            # TODO: handle collision checking, add freeing space
            task.remove_cube_collisions()

            if grasp_left.get_next_cube_pose() is None:
                grasp_left.set_next_cube_pose(next_cubes[0])
            if grasp_right.get_next_cube_pose() is None:
                grasp_right.set_next_cube_pose(next_cubes[1])
                
            if grasp_left.get_next_place_pose() is None:
                grasp_left.set_next_place_pose(behavior.get_next_place_pose_left())

            if grasp_right.get_next_place_pose() is None:
                grasp_right.set_next_place_pose(behavior.get_next_place_pose_right())

            with grasp_left.lock:
                with grasp_right.lock:
                    if grasp_left.state in [GraspState.FREE, GraspState.HAS_PICKED, GraspState.PICK_SUCCESSFUL] and grasp_right.state in [GraspState.FREE, GraspState.HAS_PICKED, GraspState.PICK_SUCCESSFUL]:
                        left_temp = copy.deepcopy(grasp_left.state)
                        right_temp = copy.deepcopy(grasp_right.state)
                        grasp_left.state = GraspState.PAUSED
                        grasp_right.state = GraspState.PAUSED
                    
                        cube_poses = task.get_cube_poses()
                        next_cube = behavior.get_next_cube_pose(cube_poses)

                        grasp_left.state = left_temp
                        grasp_right.state = right_temp

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
