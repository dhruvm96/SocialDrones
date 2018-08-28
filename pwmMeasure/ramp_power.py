# -*- coding: utf-8 -*-
#
#	 ||	____  _ __
#  +------+	  / __ )(_) /_______________ _____  ___
#  | 0xBC |	 / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+	/ /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||	/_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to the first Crazyflie found, ramps up/down
the motors and disconnects.
"""
import logging
import time
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

logging.basicConfig(level=logging.ERROR)


class MotorRampExample:
	"""Example that connects to a Crazyflie and ramps the motors up/down and
	the disconnects"""

	def __init__(self, link_uri):
		""" Initialize and run the example with the specified link_uri """

		self._cf = Crazyflie()

		self._cf.connected.add_callback(self._connected)
		self._cf.disconnected.add_callback(self._disconnected)
		self._cf.connection_failed.add_callback(self._connection_failed)
		self._cf.connection_lost.add_callback(self._connection_lost)

		self._cf.open_link(link_uri)

		print('Connecting to %s' % link_uri)

	def _connected(self, link_uri):
		""" This callback is called form the Crazyflie API when a Crazyflie has been connected and the TOCs have been downloaded."""
		self._cf.param.add_update_callback(group='controller', name='tiltComp', cb=self._param_callback)
		self._cf.param.add_update_callback(group='motorPowerSet', name=None, cb=self._param_callback)

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
			print(time.ctime())
			self._lg_pwm.start()
		except KeyError as e:
			print('Could not start log configuration,'
				  '{} not found in TOC'.format(str(e)))
		except AttributeError:
			print('Could not add PWM log config, bad configuration.')

		# Start a separate thread to do the motor test.
		# Do not hijack the calling thread!
		Thread(target=self._ramp_motors).start()

	def _param_callback(self, name, value):
		""" Callback for parameters """
		print("Readback: {0}={1}".format(name, value))

	def _pwm_log_error(self, logconf, msg):
		"""Callback from the log API when an error occurs"""
		print('Error when logging %s: %s' % (logconf.name, msg))

	def _pwm_log_data(self, timestamp, data, logconf):
		"""Callback froma the log API when data arrives"""
		print('[%d][%s]: %s' % (timestamp, logconf.name, data))

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

	def _ramp_motors(self):
		"""Disable tilt compensation"""
		self._cf.param.set_value('controller.tiltComp', '{}'.format(0))
		time.sleep(1)
		self._cf.param.set_value('motorPowerSet.enable', '{}'.format(1))
		time.sleep(1)
		self.sync_spike()
		time.sleep(1)
		motors = 0xF
		print("\n********************Test 1********************\n\n")
		self.ramp_test(0x100, 0.1, motors)
		print("\n********************Test 2********************\n\n")
		self.ramp_test(0x1000, 1, motors)
		print("\n********************Test 3********************\n\n")
		self.ramp_test(0x1000, 5, motors)
		print("\n********************Test 4********************\n\n")
		self.ramp_test(0x4000, 10, motors)
		print("\n********************Test 5********************\n\n")
		self.ramp_test(0x4000, 30, motors)

		#stop motors
		self._cf.commander.send_stop_setpoint()
		self._cf.param.set_value('motorPowerSet.enable', '{}'.format(0))
		time.sleep(1)

		# Make sure that the last packet leaves before the link is closed
		# since the message queue is not flushed before closing
		time.sleep(.1)
		self._cf.close_link()

	def sync_spike(self):
		#Sync. spike
		print("Sync spike")
		self._cf.param.set_value('motorPowerSet.m1', '{}'.format(0xFFFF))
		time.sleep(.01)
		self._cf.param.set_value('motorPowerSet.m2', '{}'.format(0xFFFF))
		time.sleep(.01)
		self._cf.param.set_value('motorPowerSet.m3', '{}'.format(0xFFFF))
		time.sleep(.01)
		self._cf.param.set_value('motorPowerSet.m4', '{}'.format(0xFFFF))
		time.sleep(.5)

		self._cf.param.set_value('motorPowerSet.m1', '{}'.format(0))
		time.sleep(.01)
		self._cf.param.set_value('motorPowerSet.m2', '{}'.format(0))
		time.sleep(.01)
		self._cf.param.set_value('motorPowerSet.m3', '{}'.format(0))
		time.sleep(.01)
		self._cf.param.set_value('motorPowerSet.m4', '{}'.format(0))
		time.sleep(.5)

	def ramp_test(self, step_pwm, delay, enable_motor):
		max_pwm = 0xFFFF
		print("Starting Ramp:")
		# Use parameters for ramp
		i = 0
		while i < max_pwm:
			modified_delay = delay
			if (enable_motor & 1):
				self._cf.param.set_value('motorPowerSet.m1', '{}'.format(i))
				time.sleep(0.02)
				modified_delay -= 0.02
			if (enable_motor & 2):
				self._cf.param.set_value('motorPowerSet.m2', '{}'.format(i))
				time.sleep(0.02)
				modified_delay -= 0.02
			if (enable_motor & 4):
				self._cf.param.set_value('motorPowerSet.m3', '{}'.format(i))
				time.sleep(0.02)
				modified_delay -= 0.02
			if (enable_motor & 8):
				self._cf.param.set_value('motorPowerSet.m4', '{}'.format(i))
				time.sleep(0.02)
				modified_delay -= 0.02
			time.sleep(modified_delay) #be more exact with total delay
			i += step_pwm
		if (enable_motor & 1):
			self._cf.param.set_value('motorPowerSet.m1', '{}'.format(max_pwm))
			time.sleep(0.02)
		if (enable_motor & 2):
			self._cf.param.set_value('motorPowerSet.m2', '{}'.format(max_pwm))
			time.sleep(0.02)
		if (enable_motor & 4):
			self._cf.param.set_value('motorPowerSet.m3', '{}'.format(max_pwm))
			time.sleep(0.02)
		if (enable_motor & 8):
			self._cf.param.set_value('motorPowerSet.m4', '{}'.format(max_pwm))
			time.sleep(0.02)
		time.sleep(delay)
		self._cf.param.set_value('motorPowerSet.m1', '{}'.format(0))
		time.sleep(0.02)
		self._cf.param.set_value('motorPowerSet.m2', '{}'.format(0))
		time.sleep(0.02)
		self._cf.param.set_value('motorPowerSet.m3', '{}'.format(0))
		time.sleep(0.02)
		self._cf.param.set_value('motorPowerSet.m4', '{}'.format(0))
		time.sleep(0.02)


if __name__ == '__main__':
	# Initialize the low-level drivers (don't list the debug drivers)
	cflib.crtp.init_drivers(enable_debug_driver=False)
	# Scan for Crazyflies and use the first one found
	print('Scanning interfaces for Crazyflies...')
	available = cflib.crtp.scan_interfaces()
	print('Crazyflies found:')
	frequency = None
	for i in available:
		print(i[0])
		if ('80' in i[0]):
			frequency = i[0]
	if len(available) > 0:
		le = MotorRampExample(frequency)
	else:
		print('No Crazyflies found, cannot run example')
