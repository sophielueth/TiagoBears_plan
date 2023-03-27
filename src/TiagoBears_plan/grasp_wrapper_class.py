from enum import Enum
from threading import Thread, Lock

import rospy

from TiagoBears_grasp.srv import PickPlace

"""
example for using both arms:
from threading import Thread

thread_left = Thread(name='grasp with left arm', target=do_something)
(...)

def do_something():
    (...)
    grasp_left.pick(cube)
    (...)

"""

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
		with self.state_lock:
			return self.state
		
	def set_state(self, state):
		with self.state_lock:
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
				with self.lock:
					self.state = GraspState.IS_PICKING
					cube_pose = self.next_cube_pose
					self.next_cube_pose = None

				# create pick request
				pick_req = rospy.ServiceProxy('pick_left' if self.is_left else 'pick_right', PickPlace)

				# query pick service
				success = None
				while success is None:
					try:
						rospy.wait_for_service('pick_left' if self.is_left else 'pick_right')
						success = pick_req(cube_pose).success

					except rospy.ServiceException as e: 
						print('Service call failed: %s'%e)
						rospy.sleep(1)

				if success:
					self.set_state(GraspState.HAS_PICKED)
				else:
					self.set_state(GraspState.FREE)

			elif state == GraspState.HAS_PICKED: # has executed a pick attempt that returned success
				# check successful pick
				# TODO
				# check_pick_req = rospy.ServiceProxy('check_pick_left' if self.is_left else 'check_pick_right', CheckPick)
				
				success = None
				while success is None:
					try:
						#rospy.wait_for_service('check_pick_left' if self.is_left else 'check_pick_right')
						#success = check_pick_req().success
						pass
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
				place_req = rospy.ServiceProxy('place_left' if self.is_left else 'place_right', PickPlace)

				# query place service
				success = None
				while success is None:
					try:
						rospy.wait_for_service('place_left' if self.is_left else 'place_right')
						success = place_req(self.get_next_place_pose()).success

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