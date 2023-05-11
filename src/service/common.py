import rospy

class EGHStatus:
	"""The IO Link interface of the gripper has different status bits that are continuously read by the daemon. EGHStatus bundles these status register values."""
	def __init__(self, blocked=None, endstop=None, success=None, referenced=None, status=None, current_position=0):
		self.blocked = blocked
		self.endstop = endstop
		self.success = success
		self.referenced = referenced
		self.status = status
		self.current_position = current_position

	def match(self, status):
		"""Checks whther given status matches this Status."""
		match_blocked = self.blocked is None or self.blocked == status.blocked
		match_endstop = self.endstop is None or self.endstop == status.endstop
		match_success = self.success is None or self.success == status.success
		match_referenced = self.referenced is None or self.referenced == status.referenced
		match_status = self.status is None or self.status == status.status
		return match_blocked and match_endstop and match_referenced and match_success and match_status
	
	def as_dict(self):
		return {
			'blocked': self.blocked,
			'endstop': self.endstop,
			'success': self.success,
			'referenced': self.referenced,
			'status': self.status,
			'current_position': self.current_position
		}

	@staticmethod
	def from_daemon(daemon):
		return EGHStatus(
			daemon.get_Blocked(),
			daemon.get_EndStop(),
			daemon.get_Success(),
			daemon.get_Referenced(),
			daemon.getActStatus(),
			daemon.get_CurrentPosition()
		)

class EGHStatusTable:
	"""The IO Link Software Manual for the gripper specifies the execution state of a command through a table with different status bit values for the states initial, failure, success. Look at a status table in the IO Link Software Manual to better understand this.
	"""
	def __init__(self, initial, success, failure):
		self.initial = initial
		self.success = success
		self.failure = failure

class CommandService:
	"""Abstracts the implementaiton of a service for a EGH command."""
	def __init__(self, service_name, service_class, on_request, on_finish, status_table, timeout=15):
		self.service_name = service_name
		self.service_class = service_class
		self.on_request = on_request
		self.on_finish = on_finish
		self.status_table = status_table
		self.timeout = timeout

	def create_service(self, daemon):
		"""Creates an actual rospy.Service from this CommandService"""
		self.daemon = daemon
		return rospy.Service(
			self.service_name,
			self.service_class,
			self._internal_service_fn
		)

	def _internal_service_fn(self, req):
		"""Wraps calling the service functionality with a loop that tracks state and handles timeout"""
		with self.daemon.lock:
			self.on_request(self.daemon, req)
			r = rospy.Rate(5)
			start_time = rospy.Time.now()
			while True:
				status = EGHStatus.from_daemon(self.daemon)
				if self.status_table.success.match(status) or self.status_table.failure.match(status) or rospy.Time.now() - start_time > rospy.Duration(self.timeout):
					return self.on_finish(self.daemon, status)
					