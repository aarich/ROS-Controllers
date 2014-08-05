#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import irobot_mudd
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import time
import math


####
# robot0.py ~ starter file for scripting the Create with ROS
####


####
# D is our global system state
####

class Data: pass    # empty class for a generic data holder
D = Data()  # an object to hold our system's services and state



####
# main and init
####


def main():
    """ the main program that gives our node a name,
       sets up service objects, subscribes to topics (with a callback),
       and then lets interactions happen!
    """
    global D

    init()

    print "Setup successful. Listening for movement commands."

    rospy.spin()

def jaws():
    D.song([56,57,30,56,57,30,64,65,30,64,65], [22,16,170,22,16,100,20,16,50,16,16,16,16,16,16]) # durations
    # rosservice call song [56,57,30,56,57,30,64,65,30,64,65] [22,16,170,22,16,100,20,16,50,16,16,16,16,16,16]


def init():
    """ returns an object (tank) that allows you
       to set the velocities of the robot's wheels
    """
    global D # to hold system state

    rospy.init_node('lab_node', anonymous=True)
    
    # we obtain the tank service
    rospy.wait_for_service('tank') # wait until the motors are available
    D.tank = rospy.ServiceProxy('tank', Tank) # D.tank is our "driver"

    # we obtain the song service
    rospy.wait_for_service('song') # wait until our voice is available
    D.song = rospy.ServiceProxy('song', Song) # D.song is our "speaker" 

    rospy.Subscriber( 'move_commands', String, callback )
    D.pub = rospy.Publisher( "move_done", String )

    # jaws()

def callback(data):
    """ This function is called for each published message
    """
    received = data.data.split()

    forward = float(received[0])
    turn =float( received[1])
    # jaws()

    # Speed:
    s = 100

    degrees_per_second = 40

    if turn  > 0.1:
        D.tank(s, -s)
        time.sleep(abs(turn / degrees_per_second ))
    elif turn < -0.1:
        D.tank(-s, s)
        time.sleep( abs(turn / degrees_per_second ))

    if forward > 0.1:
        D.tank(s, s)
        time.sleep(abs(forward * 1.2))
    elif forward < -0.1:
        D.tank(-s, -s)
        time.sleep(abs(forward * 1.2))

    D.tank(0,0)

    D.pub.publish(String("Done"))

if __name__ == "__main__":
   main()