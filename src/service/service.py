import rospy
from service.common import *
from schunk_egh_service.srv import Acknowledge, AcknowledgeResponse, Grip, GripResponse, Release, ReleaseResponse, Positioning, PositioningResponse, GetStatus, GetStatusResponse

# Here the service interfaces to the commands of the gripper are instantiated.
# The format is similar to the layout of the IO Link Software Manual.
# New commands can be implemented by copying from the manual.
_services = [
	CommandService(
		'acknowledge',
		Acknowledge,
		lambda d, r: d.cmdAcknowledge(),
		lambda d, s: AcknowledgeResponse(d.getActStatus()),
		EGHStatusTable(
			initial=EGHStatus(), 
			success=EGHStatus(status=3),
			failure=EGHStatus(status=0)
		)
	),
	CommandService(
		'grip',
		Grip,
		lambda d, r: d.cmdGrip(grip_force=r.force),
		lambda d, s: GripResponse(**(s.as_dict())),
		EGHStatusTable(
			initial=EGHStatus(referenced=1, status=3), 
			success=EGHStatus(blocked=1, endstop=0, success=1, referenced=1, status=3),
			failure=EGHStatus(blocked=1, endstop=1, success=0, referenced=1)
		)
	),
	CommandService(
		'release',
		Release,
		lambda d, r: d.cmdRelease(),
		lambda d, s: ReleaseResponse(**(s.as_dict())),
		EGHStatusTable(
			initial=EGHStatus(referenced=1, status=3), 
			success=EGHStatus(blocked=1, endstop=1, success=1, referenced=1, status=3),
			failure=EGHStatus(blocked=1, endstop=0, success=0, referenced=1)
		)
	),
	CommandService(
		'positioning',
		Positioning,
		lambda d, r: d.cmdRelative(r.target_position) if r.relative.data else d.cmdPositioning(r.target_position),
		lambda d, s: PositioningResponse(**(s.as_dict())),
		EGHStatusTable(
			initial=EGHStatus(referenced=1, status=3), 
			success=EGHStatus(blocked=0, endstop=0, success=1, referenced=1, status=3),
			failure=EGHStatus(blocked=0, endstop=None, success=0, referenced=1)
		)
	)
] 

class SchunkEGHServer():
	"""Bundles multiple service endpoints for commands on the daemon.
	Mutexes access to the daemon allowing only one service to execute a command
	at a time.
	"""
	def __init__(self, test=False):
		if test:
			import test.daemon_mock as daemon
		else:
			import daemon
		self.daemon = daemon
		self.daemon.init()
		self._services = [s.create_service(self.daemon) for s in _services]
		self.status_service = rospy.Service('get_status', GetStatus, self.get_status)

	def get_status(self):
		egh_status = EGHStatus.from_daemon(self.daemon)
		status = GetStatusResponse(**egh_status.as_dict())
		status.current_position = egh.get_CurrentPosition()
		return status

class SchunkEGHClient():
	"""Class that bundles and exposes ServiceProxies for the different commdands.
	"""
	def __init__(self):
		import schunk_egh_service.srv
		for s in _services:
			rospy.wait_for_service(s.service_name, timeout=60)
			self.__setattr__(s.service_name, rospy.ServiceProxy(s.service_name, s.service_class))
	