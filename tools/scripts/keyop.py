#!/usr/bin/env python
import rospy

from ackermann_msgs.msg import AckermannDriveStamped

import sys, select, termios, tty

banner = """
Reading from the keyboard  and Publishing to AckermannDriveStamped!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c
[,] : increase/decrease max speeds by 10%
;,'' : increase/decrease only angular speed by 10%
anything else : stop
CTRL-C to quit
"""

moveBindings = {
      'q':(1,1),
      'w':(1,0),
      'e':(1,-1),
      'a':(0,1),
      's':(0,0),
      'd':(0,-1),
      'z':(-1,-1),
      'x':(-1,0),
      'c':(-1,1)
          }

speedBindings={
      ']':(1.1,1),
      '[':(.9,1),
      ';':(1,.9),
      '\'':(1,1.1)
         }

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

speed = .2
turn = 1

def vels(speed,turn):
   return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
   settings = termios.tcgetattr(sys.stdin)
   
   pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped)
   rospy.init_node('keyop')

   x = 0
   th = 0
   status = 0

   try:
      print banner
      print vels(speed,turn)
      while(1):
         key = getKey()
         if key in moveBindings.keys():
            x = moveBindings[key][0]
            th = moveBindings[key][1]
         elif key in speedBindings.keys():
            speed = speed * speedBindings[key][0]
            turn = turn * speedBindings[key][1]

            print vels(speed,turn)
            if (status == 14):
               print msg
            status = (status + 1) % 15
         else:
            x = 0
            th = 0
            if (key == '\x03'):
               break

         msg = AckermannDriveStamped();
         msg.header.stamp = rospy.Time.now();
         msg.header.frame_id = "base_link";

         msg.drive.speed = x*speed;
         msg.drive.acceleration = 1;
         msg.drive.jerk = 1;
         msg.drive.steering_angle = th*turn
         msg.drive.steering_angle_velocity = 1

         pub.publish(msg)

   except:
      print e

   finally:
      msg = AckermannDriveStamped();
      msg.header.stamp = rospy.Time.now();
      msg.header.frame_id = "base_link";

      msg.drive.speed = 0;
      msg.drive.acceleration = 1;
      msg.drive.jerk = 1;
      msg.drive.steering_angle = 0
      msg.drive.steering_angle_velocity = 1
      pub.publish(msg)

      termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
