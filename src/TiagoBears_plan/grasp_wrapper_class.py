from enum import Enum
from threading import Thread, Lock

import rospy

from TiagoBears_grasp.srv import PickPlace
from TiagoBears_ColorDetection.srv import InitEmpty

class GraspState(Enum):
	# 0,     1,             2,      		3,   		4, 			5
	FREE, IS_PICKING, HAS_PICKED, PICK_SUCCESSFUL, IS_PLACING, PAUSED = range(6)

	

class GraspWrapper:
	move_group_execution_lock = Lock()

	def __init__(self, name, is_left):
		self.name = name
		self.is_left = is_left
		self.state = GraspState.FREE
		self.next_cube_pose = None
		self.next_place_pose = None
		self.lock = Lock()
		self.thread = Thread(name=name, target=self.run)
		self.terminate = False
		self.save_state = None

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

		print '{0} grasp_wrapper now in state {1}'.format('left' if self.is_left else 'right', state.name)

	def start(self):
		self.thread.start()

	def join(self):
		"""
		wait for the thread to stop picking or placing, then putting it in PAUSED state and returning 
		"""
		state = self.get_state()
		while state in [GraspState.IS_PICKING, GraspState.IS_PLACING]:
			rospy.sleep(0.8)
			state = self.get_state()
		self.save_state = state
		self.set_state(GraspState.PAUSED)

	def continue_(self):
		if self.save_state is not None:
			self.set_state(self.save_state)

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
				rospy.sleep(0.7)

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
							with GraspWrapper.move_group_execution_lock:
								success = pick_req(cube_pose).success

						except rospy.ServiceException as e: 
							print('Service call failed: %s'%e)
							rospy.sleep(1)

					if success:
						self.set_state(GraspState.HAS_PICKED)
						print 'pick with {0} hand was successfully executed'.format('left' if self.is_left else 'right')
					else:
						self.set_state(GraspState.FREE)
						print 'pick with {0} hand failed at execution'.format('left' if self.is_left else 'right')

			elif state == GraspState.HAS_PICKED: # has executed a pick attempt that returned success
				print '=== Checking whether recent pick with {0} hand has been successful and the cube is in the gripper ==='.format('left' if self.is_left else 'right')

				# create request to check whether the gripper is empty
				check_pick_req = rospy.ServiceProxy('/TiagoBears/is_empty_left' if self.is_left else '/TiagoBears/is_empty_right', InitEmpty)
				
				# query service
				success = None
				while success is None:
					try:
						rospy.wait_for_service('/TiagoBears/is_empty_left' if self.is_left else '/TiagoBears/is_empty_right')
						success = not check_pick_req(False).res
					except rospy.ServiceException as e: 
						print('Service call failed: %s'%e)
						rospy.sleep(1)

				if success:
					self.set_state(GraspState.PICK_SUCCESSFUL)
					print 'pick with {0} hand successfully acquired cube'.format('left' if self.is_left else 'right')
				else:
					self.set_state(GraspState.FREE)
					print "pick with {0} hand didn't acquire cube".format('left' if self.is_left else 'right')

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
						with GraspWrapper.move_group_execution_lock:
							success = place_req(place_pose).success

					except rospy.ServiceException as e: 
						print('Service call failed: %s'%e)
						rospy.sleep(1)

				# check for crash
				# TODO

				if success:
					self.set_next_place_pose(None)
					print 'place with {0} hand was successfully executed'.format('left' if self.is_left else 'right')
				else:
					print 'place with {0} hand failed at execution'.format('left' if self.is_left else 'right')


				self.set_state(GraspState.FREE)

			else:
				# is supposed to never happen
				print 'GraspWrapper: invalid state {0}'.format(state)
			
			rospy.sleep(1)

			with self.lock:
				terminate = self.terminate