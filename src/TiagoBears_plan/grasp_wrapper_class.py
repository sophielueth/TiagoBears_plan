from enum import Enum
from threading import Thread, Lock

import rospy

from TiagoBears_grasp.srv import PickPlace
from TiagoBears_ColorDetection.srv import InitEmpty


class GraspState(Enum):
	# 0,     1,             2,      		3,   		4, 			5
	FREE, IS_PICKING, HAS_PICKED, PICK_SUCCESSFUL, IS_PLACING, PAUSED = range(6)
	

class GraspWrapper:
	def __init__(self, name, is_left):
		self.name = name
		self.is_left = is_left
		self.state = GraspState.FREE
		self.next_cube_pose = None
		self.next_place_pose = None
		self.lock = Lock()
		self.thread = Thread(name=name, target=self.run)
		self.terminate = False

	def set_next_place_pose(self, pose):
		with self.lock:
			self.next_place_pose = pose

	def get_next_place_pose(self):
		with self.lock:
			return self.next_place_pose

	def set_next_cube_pose(self, pose):
		with self.lock:
			self.next_cube_pose = pose

	def get_next_cube_pose(self):
		with self.lock:
			return self.next_cube_pose

	def get_state(self):
		with self.lock:
			return self.state
		
	def set_state(self, state):
		with self.lock:
			self.state = state

	def start(self):
		self.thread.start()

	def join(self):
		self.thread.join()

	def stop(self):
		with self.lock:
			self.terminate = True

	def run(self):
		with self.lock:
			terminate = self.terminate
		
		while not terminate:
			state = self.get_state()
			if state == GraspState.PAUSED:
				# do nothing
				rospy.sleep(1)

			elif state == GraspState.FREE: # can start new pick attempt
				do_it = False
				with self.lock:
					if self.next_cube_pose is not None:
						self.state = GraspState.IS_PICKING
						cube_pose = self.next_cube_pose
						self.next_cube_pose = None
						do_it = True

				if do_it:
					print '=== Trying to pick cube at ({0}, {1}, {2}) with {3} hand ==='.format(cube_pose.position.x, cube_pose.position.y, cube_pose.position.z, 'left' if self.is_left else 'right')

					# create pick request
					pick_req = rospy.ServiceProxy('/TiagoBears/pick_left' if self.is_left else '/TiagoBears/pick_right', PickPlace)

					# query pick service
					success = None
					while success is None:
						try:
							rospy.wait_for_service('/TiagoBears/pick_left' if self.is_left else '/TiagoBears/pick_right')
							success = pick_req(cube_pose).success.data

						except rospy.ServiceException as e: 
							print('Service call failed: %s'%e)
							rospy.sleep(1)

					if success:
						self.set_state(GraspState.HAS_PICKED)
					else:
						self.set_state(GraspState.FREE)

			elif state == GraspState.HAS_PICKED: # has executed a pick attempt that returned success
				print '=== Checking whether recent pick with {0} hand has been successful and the cube is in the gripper'.format('left' if self.is_left else 'right')

				# create request to check whether the gripper is empty
				check_pick_req = rospy.ServiceProxy('/TiagoBears/is_empty_left' if self.is_left else '/TiagoBears/is_empty_right', InitEmpty)
				
				# query service
				success = None
				while success is None:
					try:
						rospy.wait_for_service('/TiagoBears/is_empty_left' if self.is_left else '/TiagoBears/is_empty_right')
						success = not check_pick_req(false).res
					except rospy.ServiceException as e: 
						print('Service call failed: %s'%e)
						rospy.sleep(1)

				if success:
					self.set_state(GraspState.PICK_SUCCESSFUL)
				else:
					self.set_state(GraspState.FREE)

			elif state == GraspState.PICK_SUCCESSFUL: # check of successfule pick was successful
				self.set_state(GraspState.IS_PLACING)
				# create place request
				place_req = rospy.ServiceProxy('/TiagoBears/place_left' if self.is_left else '/TiagoBears/place_right', PickPlace)
				place_pose = self.get_next_place_pose()

				print '=== Trying to place cube to ({0}, {1}, {2}) with {3} hand ==='.format(place_pose.position.x, place_pose.position.y, place_pose.position.z, 'left' if self.is_left else 'right')
				

				# query place service
				success = None
				while success is None:
					try:
						rospy.wait_for_service('/TiagoBears/place_left' if self.is_left else '/TiagoBears/place_right')
						success = place_req(place_pose).success.data

					except rospy.ServiceException as e: 
						print('Service call failed: %s'%e)
						rospy.sleep(1)

				# check for crash
				# TODO

				if success:
					self.set_next_place_pose(None)

				self.set_state(GraspState.FREE)

			else:
				# is supposed to never happen
				print 'GraspWrapper: invalid state {0}'.format(state)
			
			rospy.sleep(0.8)

			with self.lock:
				terminate = self.terminate