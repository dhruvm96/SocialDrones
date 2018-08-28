"""

________/\\\\\\\\\__/\\\\\\\\\\\\\\\________/\\\\\\\\\__/\\\________/\\\_
 _____/\\\////////__\/\\\///////////______/\\\////////__\/\\\_______\/\\\_
  ___/\\\/___________\/\\\_______________/\\\/___________\//\\\______/\\\__
   __/\\\_____________\/\\\\\\\\\\\______/\\\______________\//\\\____/\\\___
    _\/\\\_____________\/\\\///////______\/\\\_______________\//\\\__/\\\____
     _\//\\\____________\/\\\_____________\//\\\_______________\//\\\/\\\_____
      __\///\\\__________\/\\\______________\///\\\______________\//\\\\\______
       ____\////\\\\\\\\\_\/\\\________________\////\\\\\\\\\______\//\\\_______
        _______\/////////__\///____________________\/////////________\///________


"""
import cv2
from imutils.video import VideoStream
import imutils

import threading
import time
import queue
from collections import namedtuple
from threading import Thread

import numpy as np
import math

import cflib
from cflib.crazyflie import Crazyflie


class CFCV:
    maneuver = True
    gesture = False

    # Constants
    ideal_width = 10
    ideal_height = 10
    ideal_distance = 0.3

    default_takeoff_height = 1.25
    max_distance = 2
    max_horizontal = 1

    area_dif_threshold = 10
    x_diff_threshold = 50
    y_diff_threshold = 50

    ideal_face_width = 50
    ideal_face_width_real = 0.17
    ideal_face_height = 50
    ideal_face_height_real = 0.24

    vector_offset = 0.25

    #Resolution appears to be 640 x 465
    center_x = 319
    center_y = 233

    selfie_counter = 62

    # cap = cv2.VideoCapture(1)
    # gesture_wait_time = 10
    net = cv2.dnn.readNetFromCaffe("deploy.prototxt.txt", "res10_300x300_ssd_iter_140000.caffemodel")
    vs = VideoStream(src=1).start()

    face_cord_x = 0
    face_cord_y = 0
    face_cord_w = 0
    face_cord_h = 0

    angle = 0

    def __init__(self, link_uri):

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

    def _process_box(self, x, y, h, w):
        print("orig x: " + str(x) + " oring y: " + str(y) + " orig h: " + str(h) + " orig w: " + str(w))
        # z_off = ((CFCV.center_y + h)/2 - y) / (h/CFCV.ideal_face_height_real)
        z_off = (CFCV.center_y - (y+(h/2))) / (h/CFCV.ideal_face_height_real)
        print("z_off: " + str(z_off))
        # y_off = round(((CFCV.center_x + w)/2 - x) / (w/CFCV.ideal_face_width_real), 2)
        y_off = (CFCV.center_x - (x + (w / 2))) / (w / CFCV.ideal_face_width_real)
        print("y_off: " + str(y_off))
        x_off = round((((y_off ** 2) + (z_off ** 2)) ** (1./2.)) - CFCV.ideal_distance, 2)
        print("x_off: " + str(x_off))

        new_coords = self._translate(x_off, y_off)
        #y_off = y_off * 0.8

        if new_coords[0] > CFCV.max_horizontal:
            new_coords[0] = CFCV.max_horizontal
        if new_coords[0] < -CFCV.max_horizontal:
            new_coords[0] = -CFCV.max_horizontal

        if new_coords[1] > CFCV.max_horizontal:
            new_coords[1] = CFCV.max_horizontal
        if new_coords[1] < -CFCV.max_horizontal:
            new_coords[1] = -CFCV.max_horizontal


        return [new_coords[0], new_coords[1], z_off]

    def _translate(self, x, y):
        print("original x = " + str(x) + "\noriginal y = " + str(y) + "\noriginal angle: " + str(CFCV.angle))
        new_x = -y
        new_y = x
        theta = -math.radians(CFCV.angle)
        temp_x = new_x * math.cos(theta) + new_y * math.sin(theta)
        temp_y = -new_x * math.sin(theta) + new_y * math.cos(theta)
        if (CFCV.angle > 0) & (CFCV.angle < 90):
            temp_y = - temp_y
        if (CFCV.angle > 90) & (CFCV.angle < 180):
            temp_x = -temp_x
        return [temp_y, -temp_x]

    def _findFace(self,):
        counter = 0
        # while counter < 6:
        #     success, li = CFCV.cap.read()
        #     cv2.waitKey(30)
        #     counter += 1

        x_right = 0
        y_right = 0
        h_right = 0
        w_right = 0

        counter = 0
        while counter < 5:
            # success, li = CFCV.cap.read()
            # cv2.waitKey(30)
            # grey = cv2.cvtColor(li, cv2.COLOR_BGR2GRAY)
            # identify_face = CFCV.frontFace.detectMultiScale(grey, 1.5, 5)
            # IdentifySideFace = CFCV.BothSides.detectMultiScale(grey, 1.5, 5)
            # IdentifyProfileFace = CFCV.sideFace.detectMultiScale(grey, 1.5, 5)

            frame = CFCV.vs.read()
            if frame is None:
                counter += 1
                continue
            frame = imutils.resize(frame, width=400)
            (h, w) = frame.shape[:2]
            blob = cv2.dnn.blobFromImage(cv2.resize(frame, (640, 480)), 1.0,
                                         (640, 480), (104.0, 177.0, 123.0))
            CFCV.net.setInput(blob)
            detections = CFCV.net.forward()

            conf_final = 0

            area = 0
    
            for i in range(0, detections.shape[2]):
                confidence = detections[0, 0, i, 2]

                # confidence is strength of detection
                # IF TRUE FACE FOUND
                if confidence > 0.5:
                    # compute the (x, y)-coordinates of the bounding box for the
                    # object
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")

                    y = startY - 10 if startY - 10 > 10 else startY + 10
                    cv2.rectangle(frame, (startX, startY), (endX, endY),
                                  (0, 0, 255), 2)
                    if confidence > conf_final:
                        x_right = startX
                        y_right = startY
                        w_right = endX - startX
                        h_right = endY - startY
                        CFCV.center_x = w/2
                        CFCV.center_y = h/2
                        area = w * h
                        conf_final = confidence
            # cv2.imshow("Frame", frame)

            CFCV.face_cord_x = x_right
            CFCV.face_cord_y = y_right
            CFCV.face_cord_w = w_right
            CFCV.face_cord_h = h_right

            if area:
                self.picture = frame
                break
                # print("face coords\n x:" + str(x_right) + "y:" + str(y_right) + "w:" + str(w_right) + "z:" + str(y_right))
            counter += 1

        return [x_right, y_right, h_right, w_right]

    def _find_gesture(self, x, y, w, h):
        count = 0
        ret, img = CFCV.cap.read()
        # sub rectangle that we look at/ change to be near HEAD
        cv2.rectangle(img, (300, 300), (100, 100), (0, 255, 0), 0)
        crop_img = img[x:(x+w), y:(y+h)]
        grey = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        value = (35, 35)
        blurred = cv2.GaussianBlur(grey, value, 0)
        _, thresh1 = cv2.threshold(blurred, 127, 255,
                                   cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # findContours is OpenCV version specific
        (version, _, _) = cv2.__version__.split('.')

        if version == '3':
            image, contours, hierarchy = cv2.findContours(thresh1.copy(), \
                                                          cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        elif version == '2':
            contours, hierarchy = cv2.findContours(thresh1.copy(), cv2.RETR_TREE, \
                                                   cv2.CHAIN_APPROX_NONE)

        # contour with max area
        cnt = max(contours, key=lambda x: cv2.contourArea(x))

        #  rectangle around the contour
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(crop_img, (x, y), (x + w, y + h), (0, 0, 255), 0)

        # finding convex hull
        hull = cv2.convexHull(cnt)

        # draw contours
        drawing = np.zeros(crop_img.shape, np.uint8)
        cv2.drawContours(drawing, [cnt], 0, (0, 255, 0), 0)
        cv2.drawContours(drawing, [hull], 0, (0, 0, 255), 0)
        hull = cv2.convexHull(cnt, returnPoints=False)

        # finding defects
        defects = cv2.convexityDefects(cnt, hull)
        count_defects = 0
        cv2.drawContours(thresh1, contours, -1, (0, 255, 0), 3)
        if defects is None:
            count += 1
        else:

            for i in range(defects.shape[0]):
                s, e, f, d = defects[i, 0]

                start = tuple(cnt[s][0])
                end = tuple(cnt[e][0])
                far = tuple(cnt[f][0])

                # find length of all sides of triangle
                a = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
                b = math.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
                c = math.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)

                angle = math.acos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c)) * 57

                # ignore angles > 90 and highlight rest with red dots
                if angle <= 90:
                    count_defects += 1
                    cv2.circle(crop_img, far, 1, [0, 0, 255], -1)
                # draw a line from start to end i.e. the convex points (finger tips)
                cv2.line(crop_img, start, end, [0, 255, 0], 2)

                # run count defect 4 and 5 for open palm

            # THIS IS THE ONE FOR PEACE
            if count_defects == 1 | count_defects == 2 :
                # cv2.putText(img, " 2 Fingers Gesture Found", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, 2)
                cv2.imwrite('gesture.png', CFCV.cap.read())
                return True


                # elif(count_defects ==2):
                #   cv2.putText(img, " 3 Fingers  Gesture Found", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, 2)
            # elif (count_defects == 3):
            #    cv2.putText(img, " 4 Fingers Gesture Found", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, 2)
            # elif (count_defects == 4):
            #    cv2.putText(img, " 5 Fingers Gesture Found", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, 2)

            else:
                # cv2.putText(img, "Gesture Not Found", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, 2)
                return False

            # show appropriate images in windows
            cv2.imshow('Gesture', img)  #Todo: remove after debugging
            k = cv2.waitKey(10)

    def _on_track(self, x, y, z):
        face = self._findFace()
        new_vector = self._process_box(face[0], face[1], face[2], face[3])
        if (abs(new_vector[0] - x) > CFCV.vector_offset) | (abs(new_vector[1] - y) > CFCV.vector_offset) | (abs(new_vector[2] - z) > CFCV.vector_offset):
            return False
        return True

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
        self._comThread = CrazyComm(self._cf, self._commandQueue)
        self._comThread.start()
        print("taking off")
        self._take_off()
        time.sleep(3)

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
                    self._set_yaw(15)
                    CFCV.angle += 15
                    CFCV.angle = CFCV.angle % 360
                    time.sleep(3)

            area = face[2] * face[3]

            if area:
                # Calculate movement vector

                vector = self._process_box(face[0], face[1], face[2], face[3])
                print("Movement vector: x =" + str(vector[0]) + "y =" + str(vector[1]) + "z =" + str(vector[2]))
                distance = ((vector[0] ** 2) + (vector[1] ** 2) + (vector[2] ** 2)) ** (1./3.)
                num_movements = round((distance / CFCV.max_distance) + 0.5)
                print("Number of movements: " + str(num_movements))

                if not CFCV.maneuver:
                    cv2.imwrite('selfie.png', self.picture)
                    print("Landing")
                    self._land()
                    time.sleep(3)
                    break
                else:
                    selfie = True
                    index = 0
                    # while index < num_movements:
                    # for index in range(0, num_movements):
                    cv2.imwrite('selfie-pre.png', self.picture)
                    cv2.imwrite('selfie-pre' + str(CFCV.selfie_counter) + '.png', self.picture)
                    print("Moving along: x=" + str(vector[0]) + " y=" + str(vector[1]) + " z=" + str(vector[2]/2))
                    self._fly_velocity(vector[0], 0, 0)
                    self._fly_velocity(0, vector[1], 0)
                    # self._fly_velocity(0, 0, vector[2]/2)
                    # self._commandQueue.put(self._CrazyCommand("height", CFCV.default_takeoff_height-(vector[2]/3)))
                    # self._cur_height += vector[2]
                    time.sleep(3)

                    count_1 = 0
                    # while count_1 < 7:
                    #     success, self.picture = CFCV.cap.read()
                    #     cv2.waitKey(30)
                    #     count_1 += 1
                    # if not self._on_track(vector[0]/num_movements * (3 - index), vector[1]/num_movements * (3 - index), vector[2]/num_movements) * (3 - index):
                    #     print("Not on track! Math is wrong")
                    #     selfie = False
                    #     break
                    self.picture = CFCV.vs.read()
                    if CFCV.gesture:
                        selfie = False
                        counter = 0
                        print("looking for gestures")
                        while counter < 10:
                            if self._find_gesture(CFCV.face_cord_x-CFCV.face_cord_w,CFCV.face_cord_y,CFCV.face_cord_w,CFCV.face_cord_h):
                                print("gesture found")
                                selfie = True
                                break
                            if self._find_gesture(CFCV.face_cord_x+CFCV.face_cord_w,CFCV.face_cord_y,CFCV.face_cord_w,CFCV.face_cord_h):
                                print("gesture found")
                                selfie = True
                                break
                            k = cv2.waitKey(10)
                            counter += 1
                        print("Selfie status is " + str(selfie))
                        if selfie:
                            print("Taking selfie")
                            cv2.imwrite('selfie.png', self.picture)
                            print("Landing")
                            self._land()
                            time.sleep(2)
                        break
                    else:
                        print("Taking selfie")
                        cv2.imwrite('selfie.png', self.picture)
                        cv2.imwrite('selfie' + str(CFCV.selfie_counter) + '.png', self.picture)
                        print("Landing")
                        self._land()
                        time.sleep(2)
                        break

                        #cv2.imshow('liveImage', li)                                                                         # Todo: remove once debugged

        self._cf.commander.send_stop_setpoint()
        self._cf.close_link()

    def _take_off(self, height=None):
        height_temp = 0
        if not height:
            height = CFCV.default_takeoff_height
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
        self._cf.commander.set_client_xmode(True)

    def run(self):
        while not self.stoprequest.isSet():
            if not self._commands.empty():
                command = self._commands.get()
                value = command.value
                if command.command == "height":
                    #print("going up")
                    self._cf.commander.send_zdistance_setpoint(0, 0, 0, value)
                    self._height = value
                    # if self._height == 0:
                    #     break

                elif command.command == "turn":
                    #print("turning")
                    self._cf.commander.send_hover_setpoint(0, 0, value, self._height)

                elif command.command == "velocity":
                    #print("x:"+str(value[0])+"y:"+ str(value[1])+"z:" + str(value[2]))
                    self._cf.commander.send_velocity_world_setpoint(value[0], value[1], value[2], 0)
            else:
                #print("stopping")
                self._cf.commander.send_hover_setpoint(0, 0, 0, self._height) # try adding set_velocity

            time.sleep(self._waitTime)
        cur_height = self._height
        while cur_height > 0:
            self._cf.commander.send_zdistance_setpoint(0, 0, 0, cur_height)
            cur_height -= 0.1
            time.sleep(self._waitTime)
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

    if len(available) > 0:
        le = CFCV(available[0][0])
    else:
        print('No Crazyflies found, cannot run cfcv')
