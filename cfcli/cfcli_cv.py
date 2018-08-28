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
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt

import logging
import threading
import time
import queue
from collections import namedtuple
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie

logging.basicConfig(level=logging.ERROR)

class CrazyCLI:

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
        self._x = -1
        self._y = -1
        self._w = -1
        self._h = -1

        self._frontFace = cv2.CascadeClassifier('haarcascade_frontface.xml')
        self._sideFace = cv2.CascadeClassifier('haarcascade_profileface.xml')
        self._BothSides = cv2.CascadeClassifier('lbpcascade_profileface.xml')
        self._cap = cv2.VideoCapture(0);

        self._area_dif_threshhold = 10
        self._x_diff_threshhold = 10
        self._y_diff_threshhold = 10

        print('Connecting to %s' % link_uri)

    def _process_box(self, x_new, y_new, h_new, w_new):
        change = False
        if (self._x == -1 or self._y == -1 or self._w == -1 or self._h == -1) and (x_new > 0 and y_new > 0 and h_new > 0 and w_new > 0):
            self._comThread = CrazyComm(self._cf, self._commandQueue)
            self._comThread.start()
            print("taking off")
            self._take_off()
            change = True;
        else:
            area_dif = (((h_new * w_new) - (self._h * self._w)) / (self._h * self._w)) * 100
            x_velocity = 0.5
            duration = 0.5

            if(area_dif > 0) and (area_dif > self._area_dif_threshhold):
                print("forward")
                self._fly_velocity(x_velocity, 0, 0, duration)
                change = True
            elif(area_dif < 0 ) and ((area_dif*-1) > self._area_dif_threshhold):
                print("backward")
                self._fly_velocity(-x_velocity, 0, 0, duration)
                change = True
            else:
                x_dif = x_new - self._x
                y_dif = y_new - self._y

                y_velocity = 0
                z_velocity = 0
                if (x_dif > 0) and (x_dif > self._x_diff_threshhold):
                    y_velocity = 0.5
                    print("left")
                    change = True
                elif (x_dif < 0) and ((x_dif*-1) > self._x_diff_threshhold):
                    y_velocity = -0.5
                    print("right")
                    change = True

                if (y_dif > 0) and (y_dif > self._y_diff_threshhold):
                    print("down")
                    z_velocity = -0.5
                    change = True
                elif (y_dif < 0) and ((y_dif*-1) > self._y_diff_threshhold):
                    print("up")
                    z_velocity = 0.5
                    change = True

                self._fly_velocity(0, y_velocity, z_velocity, duration)
        if change:
            self._x = x_new
            self._y = y_new
            self._h = h_new
            self._w = w_new

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
        while True:
            sucess, li = self._cap.read()
            cv2.waitKey(30)
            grey = cv2.cvtColor(li, cv2.COLOR_BGR2GRAY)
            Identifyface = self._frontFace.detectMultiScale(grey, 1.5, 5)
            IdentifySideFace = self._BothSides.detectMultiScale(grey, 1.5, 5)
            IdentifyProfileFace = self._sideFace.detectMultiScale(grey, 1.5, 5)

            for (x, y, w, h) in Identifyface:
                # Mailbox face found
                cv2.rectangle(li, (x, y), (x + w, y + h), (255, 0, 0), 2)
                self._process_box(x,y,h,w)
                cv2.imshow('liveImage', li)
                time.sleep(0.02)
            # for (a, b, c, d) in IdentifySideFace:
            #     # mailbox face found
            #     cv2.rectangle(li, (a, b), (a + c, b + d), (155, 100, 0), 2)
            #     cv2.imshow('liveImage', li)
            # for (g, h, i, j) in IdentifyProfileFace:
            #     # Mailbox face found
            #     cv2.rectangle(li, (g, h), (g + i, h + j), (55, 200, 0), 2)
            #     cv2.imshow('liveImage', li)
            #cv2.imshow('liveImage', li)

        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        self._cf.close_link()

    def _take_off(self, height=None):
        height_temp = 0
        if not height:
            height = 0.5
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
    """ A class for a thread that refreshes a crazyflies hover command untill told to stop

        Ask the thread to stop by calling its join() method.
    """
    def __init__(self, cf, commands):
        super(CrazyComm, self).__init__()
        self._cf = cf
        self.stoprequest = threading.Event()
        self._commands = commands
        self._height = 1
        self._waitTime = 0.01

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
