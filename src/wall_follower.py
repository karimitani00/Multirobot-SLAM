#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

pub = None
RATE = 10
LIN_VEL = 0.3
DIST_THRESHOLD = 0.5
window = 0.05
NUM_ROBOTS = 2

# code adapted from Almas Abdivayev
class RobotState:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        self.scan_pub = rospy.Subscriber("base_scan", LaserScan, self.wall_callback)
        self.state = 0 # 0: stop, 1: forward
        self.error = 0
        self.error_hist = [] # list that stores all of the errors for integral term and derivative term
        self.kp = 2 # Kp
        self.kd = 1 # Kd
        self.ki = 0 # Ki
        self.rate_val = RATE
        self.side = -1 # variable that tells us whether to turn left or right
        self.rate = rospy.Rate(self.rate_val)
        self.counter = 0
        self.max_sum = 0
        self.cur_max = 0

    # callback that checks that we are at the desired distance and changes FSM if we aren't
    def wall_callback(self, msg):
        n = len(msg.ranges)

        right = min(msg.ranges[80:n/2-30]) # right
        left = min(msg.ranges[n/2+30:n-80]) # left
        front = min(msg.ranges[n/2-30:n/2:30]) # front

        if self.side == 0:
            slice = msg.ranges[200:n/2-60]
        else:
            slice = msg.ranges[n/2+60:n-200]

        self.cur_max = max(slice)

        self.counter += 1

        if self.side > -1:
            # if at least one of the increment of laserscan is looking at an empty space from either left or right side we are at juncture
            if self.cur_max > 2*self.max_sum/(self.counter -  1) or self.cur_max == msg.range_max:

                self.max_sum += self.cur_max
                if self.side == 0:
                    # turn right
                    self.state = 3
                    return
                else:
                    # turn left
                    self.state = 4
                    return

        if right < left:
            self.error = DIST_THRESHOLD - right # pos error if we want to get away from right wall
            d = right
            self.side = 0
        else:
            self.error = left - DIST_THRESHOLD # neg error if we want to get away from left wall
            d = left
            self.side = 1

        self.error_hist.append(self.error)

        # obstacle in front
        if front < DIST_THRESHOLD:
            self.state = -1
            return
        # overshoot
        if d > DIST_THRESHOLD + window:
            self.state = 1
            return
        # undershoot
        elif d < DIST_THRESHOLD - window:
            self.state = 0
            return
        # we are within the  2*window of desired distance
        elif d > DIST_THRESHOLD - window and d < DIST_THRESHOLD - window:
            self.state = 2
            return

    def derivative_term(self):
        # don't calculate derivative if we don't have first two error poitns
        if len(self.error_hist) > 2:
            prev = self.error_hist[-2]
            cur = self.error_hist[-1]
        else:
            prev = 0
            cur = 0

        return (cur-prev)/self.rate_val

    def integral_term(self):
        return sum(self.error_hist[-5:])

    def calc_error(self):
        return self.kp * self.error + self.kd * self.derivative_term() + self.ki * self.rate_val * self.integral_term() * self.rate_val

    def spin(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            cmd = Twist()
            if self.state == 0:
                # we are undershooting still getting to the goal distance
                cmd.linear.x = LIN_VEL
                cmd.angular.z = self.calc_error()

            elif self.state == 1:
                # we've overshot, so we need to move closer to the wall
                cmd.linear.x = LIN_VEL/2
                cmd.angular.z = self.calc_error()/60 # aggressive angular vel, so let's reduce it by a lot
            elif self.state == 2:
                # we are actually within some margin of desired distance so just keep moving straight
                cmd.linear.x = LIN_VEL
                cmd.angular.z = 0
            elif self.state == -1:
                # obstacle
                cmd.linear.x = 0
            elif self.state == 3:
                # turn right
                cmd.linear.x = LIN_VEL
                cmd.angular.z = -30
            elif self.state == 4:
                # turn left
                cmd.linear.x = LIN_VEL
                cmd.angular.z = 30

            self.cmd_pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("wall_follower_node")

    rs = RobotState()

    rs.spin()
