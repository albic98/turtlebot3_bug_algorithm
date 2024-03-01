#!/usr/bin/env python3

import math
import sys
# import roslib; roslib.load_manifest('bugs')
import rclpy
from rclpy.node import Node
import time
import tf_transformations as transform
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile

from location import Location, necessary_heading
from dist import Dist

current_location = Location()
current_dists = Dist()

delta = .01
WALL_PADDING = .05

STRAIGHT = 0
LEFT = 1
RIGHT = 2
MSG_STOP = 3


class Bug(Node):
    def __init__(self, algorithm, tx, ty):
        super().__init__('bug')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', location_callback, 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', sensor_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=1))
        self.tx = tx
        self.ty = ty
        self.algorithm = algorithm

    def go(self, direction):
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = 0.4
        elif direction == LEFT:
            cmd.angular.z = 0.3
        elif direction == RIGHT:
            cmd.angular.z = -0.3
        elif direction == MSG_STOP:
            pass
        
        self.pub.publish(cmd)

    def go_until_obstacle(self):
        print ("Going until destination or obstacle")
        while current_location.distance(self.tx, self.ty) > delta:
            (frontdist, _) = current_dists.get()
            if frontdist <= WALL_PADDING:
                return True

            if current_location.facing_point(self.tx, self.ty):
                self.go(STRAIGHT)
            elif current_location.faster_left(self.tx, self.ty):
                self.go(LEFT)
            else:
                self.go(RIGHT)
            time.sleep(.01)
        return False

    def follow_wall(self):
        print ("Following wall")
        while current_dists.get()[0] <= WALL_PADDING:
            self.go(RIGHT)
            time.sleep(.01)
        while not self.should_leave_wall():
            (front, left) = current_dists.get()
            if front <= WALL_PADDING:
                self.go(RIGHT)
            elif WALL_PADDING - .1 <= left <= WALL_PADDING + .1:
                self.go(STRAIGHT)
            elif left > WALL_PADDING + .1:
                self.go(LEFT)
            else:
                self.go(RIGHT)
            time.sleep(.01)

    def should_leave_wall(self):
        print ("You dolt! You need to subclass bug to know how to leave the wall")
        sys.exit(1)
    
    def bug_algorithm(self):
        # init_listener()
        print ("Calibrating sensors...")
        # This actually just lets the sensor readings propagate into the system
        time.sleep(1)
        print ("Calibrated")
        tx, ty = map(float, sys.argv[2:4])
        while current_location.distance(tx, ty) > delta:
            hit_wall = self.go_until_obstacle()
            if hit_wall:
                self.follow_wall()
        print ("Arrived at", self.tx, self.ty)


class Bug0(Bug):
    def should_leave_wall(self):
        (x, y, t) = current_location.current_location()
        dir_to_go = current_location.global_to_local(necessary_heading(x, y, self.tx, self.ty))
        at = current_dists.at(dir_to_go)
        if at > 10:
            print ("Leaving wall")
            return True
        return False

class Bug1(Bug):
    def __init__(self, algorithm, tx, ty):
        Bug.__init__(self, algorithm, tx, ty)
        self.closest_point = (None, None)
        self.origin = (None, None)
        self.circumnavigated = False

    def should_leave_wall(self):
        (x, y, t) = current_location.current_location()

        if None in self.closest_point:
            self.origin = (x, y)
            self.closest_point = (x, y)
            self.closest_distance = current_location.distance(self.tx, self.ty)
            self.left_origin_point = False
            return False
        d = current_location.distance(self.tx, self.ty)
        if d < self.closest_distance:
            print ("New closest point at", x, y)
            self.closest_distance = d
            self.closest_point = (x, y)

        (ox, oy) = self.origin
        if not self.left_origin_point and not near(x, y, ox, oy):
            # we have now left the point where we hit the wall
            print ("Left original touch point")
            self.left_origin_point = True
        elif near(x, y, ox, oy) and self.left_origin_point:
            # circumnavigation achieved!
            print ("Circumnavigated obstacle")
            self.circumnavigated = True

        (cx, ct) = self.closest_point
        if self.circumnavigated and near(x, y, cx, ct):
            self.closest_point = (None, None)
            self.origin = (None, None)
            self.circumnavigated = False
            self.left_origin_point = False
            print ("Leaving wall")
            return True

        else:
            return False

class Bug2(Bug):
    def __init__(self, algorithm, tx, ty):
        Bug.__init__(self, algorithm, tx, ty)
        self.lh = None
        self.encountered_wall_at = (None, None)

    def face_goal(self):
        while not current_location.facing_point(self.tx, self.ty):
            self.go(RIGHT)
            time.sleep(.01)

    def follow_wall(self):
        Bug.follow_wall(self)
        self.face_goal()

    def should_leave_wall(self):
        (x, y, _) = current_location.current_location()
        if None in self.encountered_wall_at:
            self.encountered_wall_at = (x, y)
            self.lh = necessary_heading(x, y, self.tx, self.ty)
            return False
        t_angle = necessary_heading(x, y, self.tx, self.ty)
        (ox, oy) = self.encountered_wall_at
        od = math.sqrt((ox-self.tx)**2 + (oy-self.ty)**2)
        cd = math.sqrt( (x-self.tx)**2 +  (y-self.ty)**2)
        dt = 0.01

        if self.lh - dt <= t_angle <= self.lh + dt and not near(x, y, ox, oy):
            if cd < od:
                print ("Leaving wall")
                return True
        return False

def location_callback(data):
    p = data.pose.pose.position

    q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
    t = transform.euler_from_quaternion(q)[2] # in [-pi, pi]
    current_location.update_location(p.x, p.y, t)

def sensor_callback(data):
    current_dists.update(data)

def near(cx, cy, x, y):
    nearx = x - .3 <= cx <= x + .3
    neary = y - .3 <= cy <= y + .3
    return nearx and neary
    

def main(args=None):

    rclpy.init(args=args)

    algorithm = sys.argv[1]
    algorithms = ["bug0", "bug1", "bug2"]
    if algorithm not in algorithms:
        print ("First argument should be one of ", algorithms, ". Was ", algorithm)
        sys.exit(1)
    if len(sys.argv) < 4:
        print ("Usage: rosrun bugs bug.py ALGORITHM X Y")
        sys.exit(1)
    (tx, ty) = map(float, sys.argv[2:4])

    print ("Setting target:",   tx, ty)

    if algorithm == "bug0":
        bug = Bug0(algorithm, tx, ty)
    elif algorithm == "bug1":
        bug = Bug1(algorithm, tx, ty)
    elif algorithm == "bug2":
        bug = Bug2(algorithm, tx, ty)

    bug.bug_algorithm()
    rclpy.spin(bug)

if __name__ == '__main__':
    main()

