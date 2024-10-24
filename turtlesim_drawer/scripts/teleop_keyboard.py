#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class TeleopKeyboard:
    def __init__(self):
        rospy.init_node('teleop_keyboard', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            print("Control Your Turtle!")
            print("Use WASD keys to move, Q to quit.")

            while not rospy.is_shutdown():
                key = self.get_key()
                twist = Twist()

                if key == 'w':
                    twist.linear.x = 2.0
                elif key == 's':
                    twist.linear.x = -2.0
                elif key == 'a':
                    twist.angular.z = 1.5
                elif key == 'd':
                    twist.angular.z = -1.5
                elif key == 'q':
                    print("Exiting Teleop Keyboard")
                    break
                else:
                    twist.linear.x = 0
                    twist.angular.z = 0

                self.velocity_publisher.publish(twist)
        except Exception as e:
            print(e)
        finally:
            twist = Twist()
            self.velocity_publisher.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    teleop = TeleopKeyboard()
    teleop.run()
