# -*- coding: utf-8 -*-
#
#	 ||		  ____  _ __
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
________/\\\\\\\\\__/\\\\\\\\\\\\\\\________/\\\\\\\\\__/\\\______________/\\\\\\\\\\\_
 _____/\\\////////__\/\\\///////////______/\\\////////__\/\\\_____________\/////\\\///__
  ___/\\\/___________\/\\\_______________/\\\/___________\/\\\_________________\/\\\_____
   __/\\\_____________\/\\\\\\\\\\\______/\\\_____________\/\\\_________________\/\\\_____
    _\/\\\_____________\/\\\///////______\/\\\_____________\/\\\_________________\/\\\_____
     _\//\\\____________\/\\\_____________\//\\\____________\/\\\_________________\/\\\_____
      __\///\\\__________\/\\\______________\///\\\__________\/\\\_________________\/\\\_____
       ____\////\\\\\\\\\_\/\\\________________\////\\\\\\\\\_\/\\\\\\\\\\\\\\\__/\\\\\\\\\\\_
        _______\/////////__\///____________________\/////////__\///////////////__\///////////__
"""
import logging
import threading
import time
import queue
import math
from collections import namedtuple
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie

logging.basicConfig(level=logging.ERROR)

class CrazyCLI:

    angle = 0

    def _translate(self, x, y):
        print("original x = " + str(x) + "\noriginal y = " + str(y) + "\noriginal angle: " + str(CrazyCLI.angle))
        new_x = -y
        new_y = x
        theta = -math.radians(CrazyCLI.angle)
        print("new x = " + str(new_x) + "\nnew y = " + str(new_y) + "\ntheta = " + str(theta))
        temp_x = (new_x * math.cos(theta)) + (new_y * math.sin(theta))
        temp_y = (-new_x * math.sin(theta)) + (new_y * math.cos(theta))
        print("cos theta = " + str(math.cos(theta)) + "\nsin theta = " + str(math.sin(theta)))
        print("final x = " + str(temp_y) + "\nfinal y = " + str(-temp_x))
        return [temp_y, temp_x]

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        self._waitTime = 0.1
        self._increment = 0.1

        self._cur_height = 0  # meters
        self._takenOff = False
        self._commandQueue = queue.Queue()
        self._CrazyCommand = namedtuple('CrazyCommand', ['command', 'value'])

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._loop).start()

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

    def _loop(self):

        command = "Init"

        while command != "done":
            user_input = input("Command:").lower().split(" ")
            command = user_input[0]

            # Parsing user input
            if len(user_input) > 1:
                velocity = float(user_input[1])
                angle = float(user_input[1])
                height = float(user_input[1])
            else:
                velocity = 1
                angle = 90
                height = None

            if len(user_input) > 2:
                duration = float(user_input[2])
            else:
                duration = 1

            #Preventing the Crazyflie from flopping around on the ground
            if self._takenOff is False:
                if command != "takeoff":
                    user_input = input("Crazyflie must take off first. Please enter \"takeoff\" to begin\n Command:").lower()
                    command = user_input[0]

                    if len(user_input) > 1:
                        velocity = float(user_input[1])
                        angle = float(user_input[1])
                        height = float(user_input[1])
                    else:
                        velocity = 1
                        angle = 90
                        height = None

                    if len(user_input) > 2:
                        duration = float(user_input[2])
                    else:
                        duration = 1

            if command == "takeoff":
                self._comThread = CrazyComm(self._cf,self._commandQueue)
                self._comThread.start()
                self._take_off(height)

            elif command == "land":
                self._land()
                self._comThread.join()
                break

            elif command == "forward":
                # coords = self._translate(velocity, 0)
                self._fly_velocity(velocity, 0ta, 0, duration)

            elif command == "backward":
                coords = self._translate(-velocity, 0)
                self._fly_velocity(coords[0], coords[1], 0, duration)

            elif command == "left":
                coords = self._translate(0, velocity)
                self._fly_velocity(coords[0], coords[1], 0, duration)

            elif command == "right":
                coords = self._translate(0, -velocity)
                self._fly_velocity(coords[0], coords[1], 0, duration)

            elif command == "up":
                self._cur_height += (velocity * duration)
                self._fly_velocity(0, 0, velocity, duration)

            elif command == "down":
                if velocity > self._cur_height:
                    self._land()
                else:
                    self._cur_height -= (velocity * duration)
                    self._fly_velocity(0, 0, -velocity, duration)

            elif command == "turn":
                self._set_yaw(angle, duration)
                CrazyCLI.angle = (CrazyCLI.angle + angle) % 360
                time.sleep(1)
                self._cf.commander.send_setpoint(0,0,0,48000)

            elif command == "turn left":
                self._set_yaw(-angle, duration)

            elif command == "done":
                if self._takenOff:
                    self._land()
                self._cf.commander.send_stop_setpoint()
                continue

            elif command == "square":
                self._fly_velocity(velocity, 0, 0, duration)
                self._fly_velocity(0, velocity, 0, duration)
                self._fly_velocity(-velocity, 0, 0, duration)
                self._fly_velocity(0, -velocity, 0, duration)

            elif command == "triangle":
                self._fly_velocity(velocity * 0.5, velocity * 0.5, 0, duration)
                self._fly_velocity(-velocity * 0.5, velocity * 0.5, 0, duration)
                self._fly_velocity(0, -velocity, 0, duration)
            elif command == "param":
                print("stabilizer.roll" + str(self._cf.param.request_param_update("stabilizer.roll")))
                print("stabilizer.yaw" + str(self._cf.param.request_param_update("stabilizer.yaw")))
            else:
                print("Invalid command")

        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(.1)
        self._cf.close_link()

    def _take_off(self, height=None):
        height_temp = 0
        if not height:
            height = 1.25
        while height_temp < height:
            self._commandQueue.put(self._CrazyCommand("height", height_temp))
            height_temp += self._increment
        self._cur_height += height
        self._takenOff = True

    def _land(self):
        height_temp = self._cur_height
        while height_temp > 0:
            self._commandQueue.put(self._CrazyCommand("height", height_temp))
            height_temp -= self._increment
        self._cur_height = 0
        self._takenOff = False

    def _set_yaw(self, angle, duration=None):
        cur_time = 0
        if not duration:
            duration = 1
        while cur_time < duration:
            self._commandQueue.put(self._CrazyCommand("turn", angle))
            cur_time += self._increment

    def _fly_velocity(self, vx, vy, vz, duration=None):
        cur_time = 0
        if not duration:
            duration = 1
        while cur_time < duration:
            self._commandQueue.put(self._CrazyCommand("velocity", [vx, vy, vz]))
            cur_time += self._increment
        self._cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)


class CrazyComm(threading.Thread):
    """ A class for a thread that refreshes a crazyflie's hover command until told to stop

        Ask the thread to stop by calling its join() method.
    """
    def __init__(self, cf, commands):
        super(CrazyComm, self).__init__()
        self._cf = cf
        self.stoprequest = threading.Event()
        self._commands = commands
        self._height = 1
        self._waitTime = 0.1

    def run(self):
        while not self.stoprequest.isSet():
            if not self._commands.empty():
                command = self._commands.get()
                value = command.value
                if command.command == "height":
                    self._cf.commander.send_zdistance_setpoint(0, 0, 0, value)
                    self._height = value

                elif command.command == "turn":
                    self._cf.commander.send_hover_setpoint(0, 0, value, self._height)

                elif command.command == "velocity":
                    self._cf.commander.send_velocity_world_setpoint(value[0], value[1], value[2], 0)
            else:
                self._cf.commander.send_hover_setpoint(0, 0, 0, self._height) # try adding set_velocity

            time.sleep(self._waitTime)
        cur_height = self._height
        while cur_height > 0:
            self._cf.commander.send_zdistance_setpoint(0, 0, 0, cur_height)
            cur_height -= 0.1

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

    if len(available) > 0:
        le = CrazyCLI(available[0][0])
    else:
        print('No Crazyflies found, cannot run example')
