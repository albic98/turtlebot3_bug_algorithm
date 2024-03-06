#!/usr/bin/env python3

import math
import sys
from threading import Thread
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

delta = 0.005
WALL_PADDING = 0.3

STRAIGHT = 0
MEDIUM_FINE_STRAIGHT = 1
FINE_STRAIGHT = 2
LEFT = 3
RIGHT = 4
MSG_STOP = 5

class Bug(Node):
    def __init__(self, algorithm, tx, ty):
        super().__init__('bug')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.sensor_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=1))
        self.tx = tx
        self.ty = ty
        self.algorithm = algorithm

    def location_callback(self, msg):
        p = msg.pose.pose.position
        q = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
        t = transform.euler_from_quaternion(q)[2] # in [-pi, pi]
        current_location.update_location(p.x, p.y, t)
        # print("Locations: ", p.x, ", ", p.y ,"\n")


    def sensor_callback(self, msg):
        current_dists.update(msg)
        # print("msg_ranges: ", msg.ranges, "\n")
        # print("Number of ranges: ", len(msg.ranges))
        # print("Front ranges: ", msg.ranges[315:360], msg.ranges[0:45], ".\n")
        # print("Left ranges: ", msg.ranges[0:180], "\n")
        # print("Front, Left: ", current_dists.get(), "\n")


    def bug_algorithm(self):
        # print ("Calibrating sensors...")
        # # This actually just lets the sensor readings propagate into the system
        # time.sleep(0.5)
        # print ("Calibrated")
        tx, ty = map(float, sys.argv[2:4])
        arrived = False
        while current_location.distance(tx, ty) > delta:
            # print("Tu sam 1.")
            hit_wall = self.go_until_obstacle()
            if hit_wall:
                # print("Tu sam 2.")
                self.follow_wall()
            if current_location.distance(tx, ty) <= delta:
                self.go(MSG_STOP)  # Stop the robot
                arrived = True
                break
            time.sleep(0.05)
        if arrived:
            print("\nArrived at: X=", self.tx, "and Y=", self.ty)

    def go(self, direction):
        (x, y, t) = current_location.current_location()
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = 0.3
        elif direction == LEFT:
            cmd.angular.z = 0.4 # * math.tanh(1.2*(necessary_heading(x, y, self.tx, self.ty)- t))
        elif direction == RIGHT:
            cmd.angular.z = -0.4 # * math.tanh(1.2*(necessary_heading(x, y, self.tx, self.ty)- t))
        elif direction == MEDIUM_FINE_STRAIGHT:
            cmd.linear.x = 0.12
        elif direction == FINE_STRAIGHT:
            cmd.linear.x = 0.04
        elif direction == MSG_STOP:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.pub.publish(cmd)

    def go_until_obstacle(self):
        print ("Going until destination or obstacle \n")
        while current_location.distance(self.tx, self.ty) > delta:
            (frontdist, _, _) = current_dists.get()
            if frontdist <= WALL_PADDING:
                # print("Tu sam 3. \n")
                return True
            if delta + 0.02 < current_location.distance(self.tx, self.ty) < delta + 0.05:
                # print("Tu sam 4. \n")
                self.go(FINE_STRAIGHT)
            if current_location.facing_point(self.tx, self.ty):
                # print("Tu sam 5. \n")
                self.go(STRAIGHT)
            elif current_location.faster_left(self.tx, self.ty):
                # print("Tu sam 6.")
                self.go(LEFT)
            else:
                # print("Tu sam 7. \n")
                self.go(RIGHT)
            time.sleep(0.05)
        return False

    def follow_wall(self):
        (x, y, t) = current_location.current_location()
        print ("Following wall \n")
        while current_dists.get()[0] <= WALL_PADDING:
            # print("Tu sam 8.")
            self.go(RIGHT)
            time.sleep(0.01)
        while not self.should_leave_wall():
            (front, left, right) = current_dists.get()
            # print("Tu sam 9.")
            if front <= WALL_PADDING:
                self.go(RIGHT)  
            elif WALL_PADDING - 0.1 <= left <= WALL_PADDING + 0.1:
                self.go(STRAIGHT)        
            elif left > WALL_PADDING + 0.1:
                self.go(LEFT)
            else:
                self.go(RIGHT)
            time.sleep(0.01)

    def should_leave_wall(self):
        print ("You dolt! You need to subclass bug to know how to leave the wall")
        sys.exit(1)
    
    def near(self, cx, cy, x, y):
        nearx = x - 0.05 <= cx <= x + 0.05
        neary = y - 0.05 <= cy <= y + 0.05
        return nearx and neary

class Bug0(Bug):
    def should_leave_wall(self):
        (x, y, t) = current_location.current_location()
        dir_to_go = current_location.global_to_local(necessary_heading(x, y, self.tx, self.ty))
        at = current_dists.at(dir_to_go)
        if at > 5:
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
        if not self.left_origin_point and not self.near(x, y, ox, oy):
            # we have now left the point where we hit the wall
            print ("Left original touch point")
            self.left_origin_point = True
        elif self.near(x, y, ox, oy) and self.left_origin_point:
            # circumnavigation achieved!
            print ("Circumnavigated obstacle")
            self.circumnavigated = True

        (cx, ct) = self.closest_point
        if self.circumnavigated and self.near(x, y, cx, ct):
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
            time.sleep(0.01)

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

        if self.lh - dt <= t_angle <= self.lh + dt and not self.near(x, y, ox, oy):
            if cd < od:
                print ("Leaving wall")
                return True
        return False


def main(args=None):

    rclpy.init(args=args)
    algorithms = ["bug0", "bug1", "bug2"]

    if len(sys.argv) < 2:
        print ("First argument should be one of ", algorithms, ".")
        sys.exit(1)
    
    algorithm = sys.argv[1]

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

    # Create a separate thread for spinning the ROS 2 node
    thread = Thread(target=rclpy.spin, args=(bug,))
    thread.start()

    try:
        while rclpy.ok():
            bug.bug_algorithm()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Exiting gracefully.\n")

    finally:
        thread.join()  # Wait for the ROS 2 node thread to finish
        bug.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


