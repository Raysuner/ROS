#!/usr/bin/env python


import rospy
from ros_arduino_python.arduino_driver import Arduino
from ros_arduino_msgs.srv import *
from ros_arduino_python.base_controller import BaseController
from geometry_msgs.msg import Twist
import os, time
import thread
from serial.serialutil import SerialException

class ArduinoROS():
    def __init__(self):
        rospy.init_node('arduino', log_level=rospy.INFO)

        # Get the actual node name in case it is set in the launch file
        self.name = rospy.get_name()

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 57600))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_footprint')
        self.motors_reversed = rospy.get_param("~motors_reversed", False)
        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        self.use_base_controller = rospy.get_param("~use_base_controller", False)

        # Set up the time for publishing the next SensorState message
        now = rospy.Time.now()

        # Initialize a Twist message
        self.cmd_vel = Twist()

        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout, self.motors_reversed)

        # Make the connection
        self.controller.connect()

        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")

        # Reserve a thread lock
        mutex = thread.allocate_lock()
        # Initialize the base controller if used
        if self.use_base_controller:
            self.myBaseController = BaseController(self.controller, self.base_frame, self.name + "_base_controller")

        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
            if self.use_base_controller:
                mutex.acquire()
                self.myBaseController.poll()
                mutex.release()

            # Publish all sensor values on a single topic for convenience
            now = rospy.Time.now()
            r.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down Arduino Node...")

        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel_pub.Publish(Twist())
            rospy.sleep(2)
        except:
            pass

        # Close the serial port
        try:
            self.controller.close()
        except:
            pass
        finally:
            rospy.loginfo("Serial port closed.")
            os._exit(0)

if __name__ == '__main__':
    try:
        myArduino = ArduinoROS()
    except SerialException:
        rospy.logerr("Serial exception trying to open port.")
        os._exit(0)
