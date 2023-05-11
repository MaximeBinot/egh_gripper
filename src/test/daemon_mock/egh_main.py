#!/usr/bin/env python2
import rospy
import time
import threading

flag_cancel = False
blocked = False
success = False
endstop = False
referenced = True
endstop_position = 40.0  # asssume inner referencing
current_position = endstop_position

thread = None

lock = threading.Lock()

def _daemon_log(string):
	rospy.logdebug("DAEMON: " + string)

def init():
	_daemon_log("Initialized mock daemon")

def cmdGrip(workpiece=0, grip_direction=0, grip_force=0,):
	_daemon_log("grip")
	global blocked
	global success
	global endstop
	blocked = 0
	success = 0
	endstop = 0
	def f():
		global blocked
		global success
		global endstop
		global current_position
		global flag_cancel
	
		while current_position > 0:
			_daemon_log(str(current_position))
			current_position -= 1
		
			if flag_cancel:
				_daemon_log("cancel grip")
				flag_cancel = False
				blocked = 1
				success = 0
				endstop = 1
				return
			
			time.sleep(0.1)

		current_position = 0.0
		blocked = 1
		success = 1
		endstop = 0

	global thread
	thread = threading.Thread(target=f, daemon=True, name="mockdaemon-grip")
	thread.start()

def cmdPositioning(target_pos):
	_daemon_log("positioning")
	global blocked
	global success
	global endstop
	global flag_cancel
	blocked = 0
	success = 0
	endstop = 0
	flag_cancel = False

	def f():
		global blocked
		global success
		global endstop
		global current_position
		global flag_cancel

		while current_position != target_pos:

			if current_position > target_pos:
				current_position -= 1
			else:
				current_position += 1

			if flag_cancel:
				_daemon_log("cancel positioning")
				flag_cancel = False
				blocked = 0
				success = 0
				endstop = 0
				return
			
			time.sleep(0.1)

		current_position = target_pos
		blocked = 0
		success = 1
		endstop = 0

	global thread
	thread = threading.Thread(target=f, daemon=True, name="mockdaemon-pos")
	thread.start()

def cmdRelative(rel_pos):
	_daemon_log("relative")
	global blocked
	global success
	global endstop
	blocked = 0
	success = 0
	endstop = 0
	def f():
		global blocked
		global success
		global endstop
		global current_position
		global flag_cancel

		rel_pos = current_position + rel_pos

		while current_position != rel_pos:

			if current_position > rel_pos:
				current_position -= 1
			else:
				current_position += 1

			if flag_cancel:
				_daemon_log("cancel relative")
				flag_cancel = False
				blocked = 0
				success = 0
				endstop = 0
				return
			
			time.sleep(0.1)

		current_position = rel_pos
		blocked = 0
		success = 1
		endstop = 0
	
	global thread
	thread = threading.Thread(target=f, daemon=True, name="mockdaemon-rel")
	thread.start()

def cmdRelease(grip_direction=0):
	_daemon_log("release")
	global blocked
	global success
	global endstop
	blocked = 0
	success = 0
	endstop = 0

	def f():
		global blocked
		global success
		global endstop
		global current_position
		global flag_cancel
	
		while current_position < endstop_position:
			current_position += 1
		
			if flag_cancel:
				_daemon_log("cancel release")
				flag_cancel = False
				blocked = 1
				success = 0
				endstop = 0
				return
			
			time.sleep(0.1)

		current_position = 40.0
		blocked = 1
		success = 1
		endstop = 1
	
	global thread
	thread = threading.Thread(target=f, daemon=True, name="mockdaemon-release")
	thread.start()

def cmdStop():
	_daemon_log("stop")
	global flag_cancel
	flag_cancel = True

def get_CurrentPosition():
	global current_position
	return current_position

def get_Blocked():
	global blocked
	return blocked

def get_Success():
	global success
	return success

def get_EndStop():
	global endstop
	return endstop

def get_Referenced():
	return 1

def getActStatus():
	return 3

acks = 0
def cmdAcknowledge():
	_daemon_log("ACK")
	global acks
	acks += 1
	if acks > 2:
		return 3
	else:
		return 0

def cmdReferencing():
	_daemon_log("referencing")
	time.sleep(1.0)
	return