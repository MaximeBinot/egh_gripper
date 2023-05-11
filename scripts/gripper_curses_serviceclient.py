#! /usr/bin/env python
import sys,os
import curses
import rospy

import service

import std_msgs

class CursesClient:

	def __init__(self):
		self.force = 0
		self.target = 0
		self.max_force = 3
		self.current_position = 0
		self.referenced = 0
		self.status = 0
		self.success = 0
		self.blocked = 0
		self.endstop = 0

		rospy.init_node("gripper_curses_client")

		self.rate = rospy.Rate(100)
	
		print("Initializing service client")
		self.client = service.SchunkEGHClient()
		print("Action client connected")

	def draw_menu(self, stdscr):
		k = 0

		# Clear and refresh the screen for a blank canvas
		stdscr.clear()
		stdscr.refresh()

		# Start colors in curses
		curses.start_color()
		curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
		curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
		curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_WHITE)
		
		statusstr = "IDLE"

		# Loop where k is the last character pressed
		while (k != ord('q')):

			# Initialization
			stdscr.clear()

			if k == curses.KEY_DOWN:
				if self.target > 0:
					self.target -= 1
			elif k == curses.KEY_UP:
				self.target += 1
			elif k == 48 or k == 49 or k == 50 or k == 51:
				self.force = k - 48
			elif k == ord('g'):
				statusstr = "Starting grip"
				resp = self.client.grip(self.force)
				statusstr = "Finished grip"
				self.current_position = resp.current_position
				self.satus = resp.status
				self.endstop = resp.endstop
				self.success = resp.success
				self.referenced = resp.referenced
				self.blocked =  resp.blocked
			elif k == ord('r'):
				statusstr = "Starting release"
				resp = self.client.release()
				statusstr = "Finished release"
				self.current_position = resp.current_position
				self.satus = resp.status
				self.endstop = resp.endstop
				self.success = resp.success
				self.referenced = resp.referenced
				self.blocked =  resp.blocked
			elif k == ord('p'):
				statusstr = "Starting positioning"
				resp = self.client.positioning(self.target, std_msgs.msg.Bool(False))
				statusstr = "Finished positioning"
				self.current_position = resp.current_position
				self.satus = resp.status
				self.endstop = resp.endstop
				self.success = resp.success
				self.referenced = resp.referenced
				self.blocked =  resp.blocked
			elif k == ord(' '):
				# self.client.cancel()
				pass
			elif k == ord('a'):
				statusstr = "Acknowledge called"
				self.status = self.client.acknowledge().status

			height, width = stdscr.getmaxyx()

			# Declaration of strings
			title = "Simple Gripper Client"[:width-1]
			subtitle = "g: grip, r: release, p: position, up/down: set target, 0-3: set force"[:width-1]
			keystr = "force: {}, target: {:2f}, current_position: {:2f}".format(self.force, self.target, self.current_position)[:width-1]
			statusbarstr = "Press 'q' to exit"
			statusbits = "status:{}, blocked:{}, endstop:{}, success:{}, referenced:{}, position: {}".format(self.status, self.blocked, self.endstop, self.success, self.referenced, self.current_position)

			# Centering calculations
			start_x_title = int((width // 2) - (len(title) // 2) - len(title) % 2)
			start_x_subtitle = int((width // 2) - (len(subtitle) // 2) - len(subtitle) % 2)
			start_x_keystr = int((width // 2) - (len(keystr) // 2) - len(keystr) % 2)
			start_y = int((height // 2) - 2)

			# Render status bar
			stdscr.attron(curses.color_pair(3))
			stdscr.addstr(height-1, 0, statusbarstr)
			stdscr.addstr(height-1, len(statusbarstr), " " * (width - len(statusbarstr) - 1))
			stdscr.attroff(curses.color_pair(3))

			# Turning on attributes for title
			stdscr.attron(curses.color_pair(2))
			stdscr.attron(curses.A_BOLD)

			# Rendering title
			stdscr.addstr(start_y, start_x_title, title)

			# Turning off attributes for title
			stdscr.attroff(curses.color_pair(2))
			stdscr.attroff(curses.A_BOLD)

			# Print rest of text
			stdscr.addstr(start_y + 1, start_x_subtitle, subtitle)
			stdscr.addstr(start_y + 3, (width // 2) - 2, '-' * 4)
			stdscr.addstr(start_y + 5, start_x_keystr, keystr)
			stdscr.addstr(start_y + 7, start_x_keystr, statusstr)
			stdscr.addstr(start_y + 9, start_x_keystr, statusbits)

			# Refresh the screen
			stdscr.refresh()

			stdscr.nodelay(True)
			k = stdscr.getch()
			self.rate.sleep()

if __name__ == "__main__":
	clt = CursesClient()
	print("Starting curses loop")
	curses.wrapper(clt.draw_menu)