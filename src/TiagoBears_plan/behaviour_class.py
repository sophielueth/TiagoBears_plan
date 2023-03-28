#!/usr/bin/env python

import copy
import rospy

from geometry_msgs.msg import Pose, Point, Quaternion

# from TiagoBears_grasp.cube_class import Cube

class Behaviour(object):
    def __init__(self, ns):
        self.ns = ns
        self._cube_length = rospy.get_param(ns + '/cube_length')
        self._stack_height_max = rospy.get_param(ns + '/stack_height')

        self._current_stack_height_left = 0
        self._current_stack_height_right = 0

        place_pos_left_start = rospy.get_param(ns + '/place_pos_left_start')
        place_pos_right_start = rospy.get_param(ns + '/place_pos_right_start')

        # TODO: change to be able to get this from pose estimation also
        self.place_pos_left_start = Point(x=place_pos_left_start[0], 
                                              y=place_pos_left_start[1], 
                                              z=place_pos_left_start[2])
        self.place_pos_right_start = Point(x=place_pos_right_start[0], 
                                               y=place_pos_right_start[1], 
                                               z=place_pos_right_start[2])
        self.place_pose_left = Pose(position=self.place_pos_left_start, orientation=Quaternion(w=1.0))
        self.place_pose_right = Pose(position=self.place_pos_right_start, orientation=Quaternion(w=1.0))

        # x values, so that x>threshold means the cube detected there is already placed (assumedly)
        self._place_threshold_left = self.place_pose_left.position.x - 0.05
        self._place_threshold_right = self.place_pose_right.position.x - 0.05

    def get_next_cube_poses(self, cube_poses):
        raise NotImplementedError
    
    def get_next_place_pose_left(self):
        raise NotImplementedError
    
    def get_next_place_pose_right(self):
        raise NotImplementedError
    
    def update_place_pose_left(self):
        raise NotImplementedError
    
    def update_place_pose_right(self):
        raise NotImplementedError

class Behaviour_stack(Behaviour):
    def __init__(self, ns):
        super(Behaviour_stack, self).__init__(ns)

    def get_next_cube_poses(self, cube_poses):
        # return pose with the smallest x value that is not already placed, if none is found return None

        next_cube_left = None
        next_cube_right = None

        # knowing cube_poses are already sorted by increasing x value
        index = 0
        while (next_cube_left is None or next_cube_right is None) and index < len(cube_poses):
            pose = cube_poses[index]

            use_left = pose.position.y > 0

            if use_left and next_cube_left is None and pose.position.x < self._place_threshold_left:
                next_cube_left = pose
            elif not use_left and next_cube_right is None and pose.position.x < self._place_threshold_right:
                next_cube_right = pose

            index += 1
        
        if next_cube_left is None and next_cube_right is None:
            return None
        
        return next_cube_left, next_cube_right

    def get_next_place_pose_left(self):
        # make sure that cube with respect the place pose is 0.5 cm above the plane to place it on
        pose_to_return = copy.deepcopy(self.place_pose_left)
        self.update_place_pose_left()

        return pose_to_return

    def get_next_place_pose_right(self):
        # make sure that cube with respect the place pose is 0.5 cm above the plane to place it on
        pose_to_return = copy.deepcopy(self.place_pose_right)
        self.update_place_pose_right()

        return pose_to_return
    
    def update_place_pose_left(self):
        # stack cubes with stack_height number of cubes
        if self._current_stack_height_left < self._stack_height_max: # add cube on top of current stack
            self.place_pose_left.position.z += self._cube_length
            self._current_stack_height_left += 1
        elif self.place_pose_left.position.y > 0.09: # start new stack in same row
            self.place_pose_left.position.y -= 0.06
            self.place_pose_left.position.z = self.place_pos_left_start.z
            self._current_stack_height_left = 1
        else:
            self.place_pose_left.position.x -= 0.06 # start a new row
            self.place_pose_left.position.y = self.place_pos_left_start.y
            self.place_pose_left.position.z = self.place_pos_left_start.z
            self._current_stack_height_left = 1

    def update_place_pose_right(self):
        # stack cubes with stack_height number of cubes
        if self._current_stack_height_right < self._stack_height_max: # add cube on top of current stack
            self.place_pose_right.position.z += self._cube_length
            self._current_stack_height_right += 1
        elif self.place_pose_right.position.y < 0.09: # start new stack in same row
            self.place_pose_right.position.y += 0.06
            self.place_pose_right.position.z = self.place_pos_right_start.z
            self._current_stack_height_right = 1
        else:
            self.place_pose_right.position.x -= 0.06 # start a new row
            self.place_pose_right.position.y = self.place_pos_right_start.y
            self.place_pose_right.position.z = self.place_pos_right_start.z
            self._current_stack_height_right = 1

    def free_space_needed(self):
        pass # TODO: implement

    def free_space(self):
        pass # TODO: implement
