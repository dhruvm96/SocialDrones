#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# This file presents an interface for interacting with the Playstation 4 Controller
# in Python. Simply plug your PS4 controller into your computer using USB and run this
# script!
#
# NOTE: I assume in this script that the only joystick plugged in is the PS4 controller.
#       if this is not the case, you will need to change the class accordingly.
#
# Copyright © 2015 Clay L. McLeod <clay.l.mcleod@gmail.com>
#
# Distributed under terms of the MIT license.

import pprint
import pygame
import time
import queue

class PS4Controller(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller = None
    axis_data = None
    button_data = None
    hat_data = None
    mailbox = None
    debug = 0

    def init(self, mailbox):
        """Initialize the joystick components"""
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.mailbox = mailbox

    def listen(self):
        """Listen for events to happen"""
        
        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        while True:
            time.sleep(.1)
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.axis_data[event.axis] = round(event.value,2)
                elif event.type == pygame.JOYBUTTONDOWN:
                    self.button_data[event.button] = True
                    if event.button == 0:
                        self.mailbox.put("square")
                    elif event.button == 1:
                        self.mailbox.put("cross")
                    elif event.button == 2:
                        self.mailbox.put("circle")
                    elif event.button == 3:
                        self.mailbox.put("triangle")
                    elif event.button == 4:
                        self.mailbox.put("L1")
                    elif event.button == 5:
                        self.mailbox.put("R1")
                    elif event.button == 8:
                        self.mailbox.put("select")
                    elif event.button == 9:
                        self.mailbox.put("start")
                    #0: square
                    #1: cross
                    #2: circle
                    #3: triangle
                    #4: L1
                    #5: R1
                    #8: Share
                    #9: Options
                    if self.debug == 1: print(event.button)
                elif event.type == pygame.JOYBUTTONUP:
                    self.button_data[event.button] = False
                elif event.type == pygame.JOYHATMOTION:
                    self.hat_data[event.hat] = event.value

if __name__ == "__main__":
    ps4 = PS4Controller()
    ps4.init(queue.Queue())
    ps4.debug = 1
    ps4.listen()
