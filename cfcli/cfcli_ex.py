# -*- coding: utf-8 -*-
#
#	 ||	____  _ __
#  +------+	  / __ )(_) /_______________ _____	___
#  | 0xbc |	 / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+	/ /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#	||	||	/_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  copyright (c) 2014 bitcraze ab
#
#  crazyflie nano quadcopter client
#
#  this program is free software; you can redistribute it and/or
#  modify it under the terms of the gnu general public license
#  as published by the free software foundation; either version 2
#  of the license, or (at your option) any later version.
#
"""	
-------------------------------
		Team 9 - Social Drones
-------------------------------
		Kushantha Attanayake
		Manuel Navarro Catalán
		Dhruv Mathew
		Abhinav Mohan
		Trevor Murdock
		Akilan Pughazhendi
		Michael Smith
"""		
"""
________/\\\\\\\\\___/\\\\\\\_____________________
 _____/\\\////////___\ \\\\\_______________________
  ___/\\\/____________\ \\\__________________________
   __/\\\______________\/_/___________________________
	_\/\\\_____________________________________________
	 _\//\\\_____________/\\\_______/\\\\\\\___/\\\_\\\_  
	  __\///\\\__________\ \\\______\ \\\\\\___\/_/\\\/__ 
	   ____\////\\\\\\\\\_\ \\\\\\\__\ \\\\\\\___/\\\/\\\_
		_______\/////////__\/_____/___\/_____/___\/_/\/_/__
"""
import os
import logging
import threading
from threading import Thread
import time
import queue
from collections import namedtuple
from cflib.crazyflie.log import LogConfig
import cflib
from cflib.crazyflie import Crazyflie
import argparse

logging.basicConfig(level=logging.ERROR)

# use python3 cfcli_ex.py -h to view options
parser = argparse.ArgumentParser(description='crazyflie command-line interface extended')
parser.add_argument('-i','--inputmethod', dest='inputmethod', default='cli', help='Default cli')
parser.add_argument('-l','--log', dest='log',  nargs='?', const='log', default=0, help='Enable logging. Default file "log"')
parser.add_argument('-t','--takeoffheight', dest='takeoffheight', default=0.5, help='Initial height to fly to after takeoff')
parser.add_argument('-f','--frequency', dest='frequency', default='radio://0/80/250K', help='Drone radio frequency')
args = parser.parse_args()

# contains callbacks for crazyflie api and main loops for different inputmethods.
class CrazyCLI:

	def __init__(self, link_uri):
		""" Initialize and run the example with the specified link_uri """

		self._cf = Crazyflie()

		self._cf.connected.add_callback(self._connected)
		self._cf.disconnected.add_callback(self._disconnected)
		self._cf.connection_failed.add_callback(self._connection_failed)
		self._cf.connection_lost.add_callback(self._connection_lost)

		self._cf.open_link(link_uri)

		self.INPUT_METHOD = args.inputmethod
		self.LOG_FILENAME = args.log
		self._init_height = float(args.takeoffheight)
		self._cur_height = 0
		self._takenOff = False
		self._commandQueue = queue.Queue()
		self._CrazyCommand = namedtuple('CrazyCommand', ['command', 'value'])

		print('Connecting to %s' % link_uri)

	# Radio connection callbacks
	def _connected(self, link_uri):
		""" This callback is called form the Crazyflie API when a Crazyflie
		has been connected and the TOCs have been downloaded."""

		if (self.LOG_FILENAME):
			self.log_file = open(self.LOG_FILENAME, 'w')
			self._lg_pwm = LogConfig(name='PWM', period_in_ms=50)
			self._lg_pwm.add_variable('pwm.m1_pwm', 'float')
			self._lg_pwm.add_variable('pwm.m2_pwm', 'float')
			self._lg_pwm.add_variable('pwm.m3_pwm', 'float')
			self._lg_pwm.add_variable('pwm.m4_pwm', 'float')
			self._lg_pwm.add_variable('pm.vbat', 'float')

			# Adding the configuration cannot be done until a Crazyflie is
			# connected, since we need to check that the variables we
			# would like to log are in the TOC.
			try:
				self._cf.log.add_config(self._lg_pwm)
				# This callback will receive the data
				self._lg_pwm.data_received_cb.add_callback(self._pwm_log_data)
				# This callback will be called on errors
				self._lg_pwm.error_cb.add_callback(self._pwm_log_error)
				# Start the logging and give timestamp
				self.log_file.write("%s\n" % time.ctime())
				self._lg_pwm.start()
			except KeyError as e:
				print('Could not start log configuration,'
					  '{} not found in TOC'.format(str(e)))
			except AttributeError:
				print('Could not add PWM log config, bad configuration.')

		# Start a separate thread to do the motor test.
		# Do not hijack the calling thread!
		if self.INPUT_METHOD == "cli":
			Thread(target=self._parse_cli).start()
		elif self.INPUT_METHOD == "ps4":
			Thread(target=self._parse_ps4).start()
		elif self.INPUT_METHOD == "cv":
			Thread(target=self._parse_cv).start()
		else:
			Thread(target=self._parse_file).start()

	def _pwm_log_error(self, logconf, msg):
		"""Callback from the log API when an error occurs"""
		print('Error when logging %s: %s' % (logconf.name, msg))

	def _pwm_log_data(self, timestamp, data, logconf):
		"""Callback froma the log API when data arrives"""
		self.log_file.write("[%d][%s]: %s\n" % (timestamp, logconf.name, data))

	def _connection_failed(self, link_uri, msg):
		"""Callback when connection initial connection fails (i.e no Crazyflie
		at the specified address)"""
		print('Connection to %s failed: %s' % (link_uri, msg))

	def _connection_lost(self, link_uri, msg):
		"""Callback when disconnected after a connection has been made (i.e
		Crazyflie moves out of range)"""
		print('Connection to %s lost: %s' % (link_uri, msg))

	def _disconnected(self, link_uri):
		"""Callback when the Crazyflie is disconnected (called in all cases)"""
		print('Disconnected from %s' % link_uri)
		if (self.LOG_FILENAME):
			self.log_file.close 

	#interface between command line and _interpret_command()
	def _parse_cli(self):
		print("Crazyflie is ready for commands.")
		command = "Init"
		while command != "done":
			user_input = input("Command:").lower().split(" ")

			#parsing
			command = user_input[0]
			if len(user_input) > 1:
				velocity = float(user_input[1])
				angle = float(user_input[1])
				height = float(user_input[1])
			else:
				velocity = 1
				angle = 90
				height = 1
			if len(user_input) > 2:
				duration = float(user_input[2])
			else:
				duration = 1

			self._interpret_command(command, velocity, angle, height, duration)
	
	#interface between controller and _interpret_command()
	def _parse_ps4(self):
		from controller import PS4Controller

		#It is not necessary to start a new thread for this
		#But since the controller is overhead anyway, it is elegant enough
		ps4 = PS4Controller()
		self.ps4_queue = queue.Queue()
		ps4.init(self.ps4_queue)
		self.controller_thread = Thread(target=ps4.listen)
		self.controller_thread.start()
		print("Crazyflie is ready for commands.")
		button = "Init"
		command = "Init"
		while command != "done":
			if not self.ps4_queue.empty():
				button = self.ps4_queue.get()
				if button == "select":
					command = "done"
				elif button == "start" and self._takenOff == False:
					command = "takeoff"	
				elif button == "start" and self._takenOff == True:
					command = "land"
				elif button == "square":
					command = "square"
				elif button == "triangle":
					command = "triangle"
				self._interpret_command(command)
		self.controller_thread.join()
	
	#interface between computer vision and _interpret_command()
	def _parse_cv(self):
		import cv2
		maneuver = False

		# Constants
		self.ideal_width = 10
		self.ideal_height = 10
		self.ideal_distance = 1
		self.max_distance = 2
		self.area_dif_threshold = 10
		self.x_diff_threshold = 50
		self.y_diff_threshold = 50
		self.ideal_face_width = 50
		self.ideal_face_width_real = 0.17
		self.ideal_face_height = 50
		self.ideal_face_height_real = 0.24
		self.vector_offset = 0.25
		#Resolution appears to be 640 x 465
		self.center_x = 319
		self.center_y = 233
		self.frontFace = cv2.CascadeClassifier('haarcascade_frontface.xml')
		self.cap = cv2.VideoCapture(1)

		self._interpret_command("takeoff")
		time.sleep(1)

		while True:

			# Search
			face_found = False
			face = None
			print("Looking for face")
			while not face_found:
				face = self._findFace()
				if (face[0] + face[1] + face[2] + face[3]) > 0:
					face_found = True
					print("Face Found")
				else:
					print("No face found, rotating")
					self._interpret_command(command='turnright', angle=15)
					time.sleep(3)						  #Delay to account for camera delay

			area = face[2] * face[3]

			if area:
				# Calculate movement vector
				vector = self._process_box(face[0], face[1], face[2], face[3])
				print("Movement vector: x =" + str(vector[0]) + "y =" + str(vector[1]) + "z =" + str(vector[2]))
				distance = ((vector[0] ** 2) + (vector[1] ** 2) + (vector[2] ** 2)) ** (1./3.)
				num_movements = round((distance / self.max_distance) + 0.5)
				print("Number of movements: " + str(num_movements))

				if not self.maneuver:
					cv2.imwrite('selfie.png', self.picture)
					print("Landing")
					self._land()
					time.sleep(3)
					break
				else:
					selfie = True

					for index in range(0, num_movements):
						self._interpret_command(command='velocityvector', velocity=[vector[0]/num_movements, vector[1]/num_movements, vector[2]/num_movements])
						time.sleep(0.1)
						if not self._on_track(vector[0]/num_movements * (3 - index), vector[1]/num_movements * (3 - index), vector[2]/num_movements) * (3 - index):
							print("Not on track! Math is wrong")
							selfie = False
							break
					if selfie:
						print("Taking selfie")
						cv2.imwrite('selfie.png', self.cap.read())
						print("Landing")
						self._interpret_command(command='land')
						time.sleep(3)
						break
			#cv2.imshow('liveImage', li)																		 # Todo: remove once debugged
		self._interpret_command('done')

	#called by parse_cv()
	def _process_box(self, x, y, h, w):
		z_off = ((self.center_y + h)/2 - y) / (h/self.ideal_face_height_real)
		y_off = ((self.center_x + w)/2 - x) / (w/self.ideal_face_width_real)
		x_off = (((y_off ** 2) + (z_off ** 2)) ** (1./2.)) - self.ideal_distance
		return [y_off, x_off, z_off]

	#called by parse_cv()
	def _on_track(self, x, y, z):
		face = self._findFace()
		new_vector = self._process_box(face[0], face[1], face[2], face[3])
		if (abs(new_vector[0] - x) > CFCV.vector_offset) | (abs(new_vector[1] - y) > CFCV.vector_offset) | (abs(new_vector[2] - z) > CFCV.vector_offset):
			return False
		return True

	#interface between file and _interpret_command()
	def _parse_file(self):
		if not os.path.isfile(args.inputmethod):
			print("%s either does not exist or is a directory" % args.inputmethod)
			self._interpret_command("done")
		else:
			input_file = open(args.inputmethod, 'r')
			settings = input_file.readline().split(" ")
			velocity = float(settings[0])
			angle = float(settings[1])
			height = float(settings[2])
			duration  = float(settings[3])
			for line in input_file:
				instruction = line.split(" ")
				command = instruction[0]
				delay = float(instruction[1])
				self._interpret_command(command, velocity, angle, height, duration)
				time.sleep(delay)
			input_file.close

	# turn high-level commands into low-level movement and/or sequence of other commands
	def _interpret_command(self, command, velocity=1, angle=90, height=0.1, duration=1):
		if (self.LOG_FILENAME):
			self.log_file.write("###Command: %s###\n" % command)
		if command == "done":
			if self._takenOff:
				self._interpret_command("land")
			time.sleep(5)
			if (self.LOG_FILENAME):
				self.log_file.close 
			self._cf.close_link()

		#Keep crazyflie from flopping around on ground
		elif  self._takenOff == False and command != "takeoff":
			print("Crazyflie must take off first. Please enter \"takeoff\" to begin.")

		elif  self._takenOff == True and command == "takeoff":
			print("Crazyflie has already taken off.")

		elif command == "takeoff":
			self._takenOff = True
			self._comThread = CrazyComm(self._cf,self._commandQueue)
			self._comThread.start()
			self._set_height_change(self._init_height, duration)
			self.cur_height = self._init_height

		elif command == "land":
			self._takenOff = False
			self._set_height_change(-self._cur_height, duration)
			self._comThread.join()

		elif command == "forward":
			self._set_velocity(velocity, 0, 0, duration)

		elif command == "backward":
			self._set_velocity(-velocity, 0, 0, duration)

		elif command == "left":
			self._set_velocity(0, velocity, 0, duration)

		elif command == "right":
			self._set_velocity(0, -velocity, 0, duration)

		elif command == "up":
			self._cur_height += height 
			self._set_height_change(height, duration) 

		elif command == "down":
			if height > self._cur_height: 
				self._interpret_command("land")
			else:
				self._cur_height -= height 
				self._set_height_change(-height, duration) 

		elif command == "velocityvector":
			self._set_velocity(velocity[0], velocity[1], velocity[2], duration)
			self._cur_height += velocity[2] 

		elif command == "turnright":
			self._set_yaw_v(angle, duration)

		elif command == "turnleft":
			self._set_yaw_v(-angle, duration)

		elif command == "square":
			self._set_velocity(velocity, 0, 0, duration)
			self._set_velocity(0, velocity, 0, duration)
			self._set_velocity(-velocity, 0, 0, duration)
			self._set_velocity(0, -velocity, 0, duration)

		elif command == "triangle":
			self._set_velocity(velocity * 0.5, velocity * 0.5, 0, duration)
			self._set_velocity(-velocity * 0.5, velocity * 0.5, 0, duration)
			self._set_velocity(0, -velocity, 0, duration)
		else:
			print("Invalid command")

	# These methods are low-level interfaces to CrazyComm.
	def _set_height_change(self, height, duration):
		self._commandQueue.put(self._CrazyCommand("height", [duration, height]))

	def _set_yaw_v(self, angle, duration):
		self._commandQueue.put(self._CrazyCommand("turn", [duration, angle]))

	def _set_velocity(self, vx, vy, vz, duration):
		self._commandQueue.put(self._CrazyCommand("velocity", [duration, vx, vy, vz]))

# CrazyComm is the first thing we need to migrate to the crazyflie after
# changing _cf.commander functions into direct commands to the motor interface
class CrazyComm(threading.Thread):
	""" A class for a thread that refreshes a crazyflies hover command untill told to stop
		Ask the thread to stop by calling its join() method.
	"""
	def __init__(self, cf, commands_queue):
		super(CrazyComm, self).__init__()
		self._cf = cf
		self.stoprequest = threading.Event()
		self._commands_queue = commands_queue
		self._height = 0
		self._waitTime = 0.1

	def run(self):
		while not self.stoprequest.isSet():
			if not self._commands_queue.empty():
				command_tuple = self._commands_queue.get()
				command = command_tuple.command
				value = command_tuple.value
				cur_time = 0
				#value step increment
				i1 = value[1] * self._waitTime / value[0]
				# For this to work, duration must always be index 0 in the value
				while cur_time < value[0]:
					if command == "height":
						self._height += i1
						self._cf.commander.send_zdistance_setpoint(0, 0, 0, self._height)
					elif command == "turn":
						self._cf.commander.send_hover_setpoint(0, 0, value[1], self._height)
					elif command == "velocity":
						self._cf.commander.send_velocity_world_setpoint(value[1], value[2], value[3], 0)
					cur_time += self._waitTime
					time.sleep(self._waitTime)
			else: #hover if no command
				self._cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)
				time.sleep(self._waitTime / 2)
				self._cf.commander.send_zdistance_setpoint(0, 0, 0, self._height)
				time.sleep(self._waitTime / 2)

		# after stop request made, land
		while self._height > 0:
			self._cf.commander.send_zdistance_setpoint(0, 0, 0, self._height)
			self._height -= 0.1
			time.sleep(self._waitTime * 2)
		time.sleep(1)
		self._cf.commander.send_stop_setpoint()

	def join(self, timeout=None):
		self.stoprequest.set()
		super(CrazyComm, self).join(timeout)

if __name__ == '__main__':
	# Initialize the low-level drivers (don't list the debug drivers)
	cflib.crtp.init_drivers(enable_debug_driver=False)
	# Scan for Crazyflies and use the first one found
	print('Scanning interfaces for Crazyflies...')
	available = cflib.crtp.scan_interfaces()
	print('Crazyflies found:')
	for i in available:
		print(i[0])

		if i[0] == args.frequency:
			le = CrazyCLI(i[0])
			break
	else:
		print('No Crazyflies found, cannot run example')
