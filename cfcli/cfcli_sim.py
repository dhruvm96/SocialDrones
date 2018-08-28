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
	 _\//\\\_____________
	  __\///\\\__________
	   ____\////\\\\\\\\\_Sim
		_______\/////////__
"""

#This is a modified version of cfcli_ex meant to run on the raspberry pi 0w 
# for controlling drone in AirSim

import os
import threading
from threading import Thread
import time
import argparse
from AirSimClient import *
import cv2
import numpy

parser = argparse.ArgumentParser(description='crazyflie command-line interface extended')
parser.add_argument('-i','--inputmethod', dest='inputmethod', default='cli', help='Default cli')
parser.add_argument('-t','--takeoffheight', dest='takeoffheight', default=0.5, help='Initial height to fly to after takeoff')
parser.add_argument('-p', '--ip', dest='ip', default='127.0.0.1', help='Ip address to connect to simulator')
args = parser.parse_args()

class CrazyCLI:

	def __init__(self):
		""" Initialize and run the example with the specified link_ip """

		print('Connecting to %s' % args.ip)
		self.client = MultirotorClient(ip = args.ip)
		self.client.confirmConnection()
		self.client.enableApiControl(True)
		self.client.armDisarm(True)

		self._init_height = float(args.takeoffheight)
		self._cur_height = 0
		self._takenOff = False

		if args.inputmethod == "cli":
			Thread(target=self._parse_cli).start()
		elif args.inputmethod == "cv":
			Thread(target=self._parse_cv).start()
		else:
			Thread(target=self._parse_file).start()

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

	def _findFace(self,):
		responses = self.client.simGetImages([ImageRequest(1, AirSimImageType.Scene, pixels_as_float=False, compress=False)])
		li = numpy.asmatrix(numpy.fromstring(string=responses[0].image_data_uint8, dtype='uint8'))
		success = True
		cv2.waitKey(30)
		#grey = cv2.cvtColor(li, cv2.COLOR_BGR2GRAY)
		grey = li
		identify_face = self.frontFace.detectMultiScale(grey, 1.5, 5)

		x_right = 0
		y_right = 0
		h_right = 0
		w_right = 0
		area = 0
		for (x, y, w, h) in identify_face:
			if h * w > area:
				x_right = x
				y_right = y
				w_right = w
				h_right = h
				area = h * w
		cv2.rectangle(li, (x_right, y_right), (x_right + w_right, y_right + h_right), (255, 0, 0), 2)
		#cv2.imshow('liveImage', li)
		if area:
			self.picture = li
			print("face coords\n x:" + str(x_right) + "y:" + str(y_right) + "w:" + str(w_right) + "z:" + str(y_right))

		return [x_right, y_right, h_right, w_right]

	def _process_box(self, x, y, h, w):
		z_off = ((self.center_y + h)/2 - y) / (h/self.ideal_face_height_real)
		y_off = ((self.center_x + w)/2 - x) / (w/self.ideal_face_width_real)
		x_off = (((y_off ** 2) + (z_off ** 2)) ** (1./2.)) - self.ideal_distance
		return [y_off, x_off, z_off]

	def _on_track(self, x, y, z):
		face = self._findFace()
		new_vector = self._process_box(face[0], face[1], face[2], face[3])
		if (abs(new_vector[0] - x) > CFCV.vector_offset) | (abs(new_vector[1] - y) > CFCV.vector_offset) | (abs(new_vector[2] - z) > CFCV.vector_offset):
			return False
		return True

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

	# turn high-level commands into low-level commands
	def _interpret_command(self, command, velocity=1, angle=90, height=0.1, duration=1):
		if command == "done":
			if self._takenOff:
				self._interpret_command("land")
			time.sleep(5)

		#Keep crazyflie from flopping around on ground
		elif  self._takenOff == False and command != "takeoff":
			print("Crazyflie must take off first. Please enter \"takeoff\" to begin.")

		elif  self._takenOff == True and command == "takeoff":
			print("Crazyflie has already taken off.")

		elif command == "takeoff":
			self._takenOff = True
			vz = self._init_height / duration
			self._set_velocity(0, 0, vz, duration)

		elif command == "land":
			self._takenOff = False
			vz = -self._cur_height / duration
			self._set_velocity(0, 0, vz, duration)

		elif command == "forward":
			self._set_velocity(velocity, 0, 0, duration)

		elif command == "backward":
			self._set_velocity(-velocity, 0, 0, duration)

		elif command == "left":
			self._set_velocity(0, velocity, 0, duration)

		elif command == "right":
			self._set_velocity(0, -velocity, 0, duration)

		elif command == "up":
			vz = height / duration
			self._set_velocity(0, 0, vz, duration) 

		elif command == "down":
			vz = -height / duration
			self._set_velocity(0, 0, vz, duration) 

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

	# These methods are the interfaces to the AirSim pyclient.
	def _set_height(self, duration):
		self.client.moveByVelocityZ(0, 0, self._cur_height, duration)
		return

	def _set_yaw_v(self, angle, duration):
		self.client.rotateByYawRate(angle, duration)
		return

	def _set_velocity(self, vx, vy, vz, duration):
		self.client.moveByVelocity(vx, vy, -vz, duration)
		return

if __name__ == '__main__':
	CrazyCLI()
