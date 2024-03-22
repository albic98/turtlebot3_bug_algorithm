#!/usr/bin/env python3

import math
import sys
import random
from threading import Thread
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

DISTANCE = 0.3

STRAIGHT = 0
MEDIUM_FINE_STRAIGHT = 1
LEFT = 2
RIGHT = 3
FINE_LEFT = 4
FINE_RIGHT = 5
MSG_STOP = 6
BACKWARDS = 7

class Bug(Node):
    def __init__(self, algorithm, tx, ty):
        super().__init__('bug')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.sensor_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=1))
        self.tx = tx
        self.ty = ty
        self.algorithm = algorithm
    
    ## Function for determining the position of the robot using the "\odom" topic
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

    ## Function for determining the distance of the robot from the wall and/or the obstacles
    def sensor_callback(self, msg):
        current_dists.update(msg)

    ## Main function which calls other functions necessary for successfully avoiding obstacles
    def bug_algorithm(self):
        tx, ty = map(float, sys.argv[2:4])
        arrived = False
        while current_location.distance(tx, ty) > delta:
            hit_wall = self.go_until_obstacle()
            if hit_wall:
                self.follow_wall()
            elif current_location.distance(tx, ty) <= delta:
                self.go(MSG_STOP)  # Stop the robot
                arrived = True
                break
            else:
                break
            time.sleep(0.05)
        if arrived:
            print("\nArrived at: X= ", self.tx, " and Y= ", self.ty, "\n")
            time.sleep(1.0)
            print("\nPlace the robot on a different location or shutdown program.")

    ## Function which publishes the velocity input on the "/cmd_vel" topic, depending on which one is required
    def go(self, direction):
        (x, y, t) = current_location.current_location()
        n = necessary_heading(x,y, self.tx, self.ty)
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = 0.4 * math.tanh(1.2 * current_location.distance(self.tx, self.ty))
        elif direction == MEDIUM_FINE_STRAIGHT:
            cmd.linear.x = 0.2
        elif direction == BACKWARDS:
            cmd.linear.x = -0.15
        elif direction == LEFT:
            cmd.angular.z = 0.7
        elif direction == RIGHT:
            cmd.angular.z = -0.7
        elif direction == FINE_LEFT:
            cmd.angular.z = 0.45 * abs(math.tanh(1.8 * (n - t)))
        elif direction == FINE_RIGHT:
            cmd.angular.z = -0.45 * abs(math.tanh(1.8 * (n - t)))
        elif direction == MSG_STOP:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.pub.publish(cmd)

    ## Defines the robot behaviour when there are no obstacles around
    def go_until_obstacle(self):
        print ("\nGoing until destination or obstacle")
        self.obstacle_time = 0.0
        while current_location.distance(self.tx, self.ty) > delta:
            (x, y, t) = current_location.current_location()
            n = necessary_heading(x,y, self.tx, self.ty)
            (frontdist, _, _, _, _) = current_dists.get()
            if frontdist <= DISTANCE:
                return True
            # This if condition checks if the robot's heading 'n' is close to the boundary conditions of -pi and pi.
            # This check is essential to prevent erratic behavior when the robot's position approaches these boundary values.
            # If the heading is within a small range around these boundary values, the robot performs corrective actions to ensure smooth navigation.
            if math.pi - 0.02 <= n <= math.pi + 0.02 or -math.pi - 0.02 <= n <= -math.pi + 0.02:
                if math.pi - 0.02 <= n <= math.pi + 0.02:
                    self.go(STRAIGHT)
                    time.sleep(0.3)
                    self.go(FINE_LEFT)
                    time.sleep(0.5)
                if math.pi - 0.02 <= n <= math.pi + 0.02:
                    self.go(STRAIGHT)
                    time.sleep(0.3)
                    self.go(FINE_RIGHT)
                    time.sleep(0.5)
            if current_location.facing_point(self.tx, self.ty):
                self.go(STRAIGHT)
            elif current_location.faster_left(self.tx, self.ty):
                self.go(FINE_LEFT)
            else:
                if  n < 0 and (math.pi/3 < t < math.pi or -math.pi/2 - 0.05 < t < -math.pi/2 + 0.05):
                    self.go(FINE_LEFT)
                else:
                    self.go(FINE_RIGHT)
            time.sleep(0.01)
        return False


####################################################
            ### Bug 0 STARTS HERE ###
####################################################
        
class Bug0(Bug):
    def __init__(self, algorithm, tx, ty):
        Bug.__init__(self, algorithm, tx, ty)

    ## Defines the robot behaviour around walls and/or obstacles
    def follow_wall(self):
        print("\nFollowing wall")
        self.right_turn = False # this two flags define the direction of the previous turn
        self.left_turn = False  # enabling the robot to perfrom the next action correctly
        stuck_time_1 = 0.0
        stuck_time_2 = 0.0
        stuck_time_3 = 0.0

        # This loop executes when the value of DISTANCE is less than the default setting.
        # It determines the direction for the robot to turn based on the relative position of the wall.
        # The loop continues until the DISTANCE is restored to its default value.
        while current_dists.get()[0] <= DISTANCE:
            (x, y, t) = current_location.current_location()
            (front, left, right, backleft, backright) = current_dists.get()
            if (backleft < backright or left < right) and stuck_time_1 <= 2:
                if backleft < backright:
                    self.go(RIGHT)
                    stuck_time_1 += 0.01
                else:
                    self.go(LEFT)
                    stuck_time_1 += 0.01
            elif (backleft > backright or left > right) and stuck_time_1 <= 2:
                print("Tu sam 2.3.")
                self.go(LEFT)
                stuck_time_1 += 0.01
            else:
                if self.ty <= 0:
                    if  left < right and self.tx < x:
                        self.go(RIGHT)
                    else:
                        if y*self.ty > 0:
                            self.go(LEFT)
                        else:
                            self.go(RIGHT)
                else:
                    if  left < right and self.tx > x:
                        self.go(RIGHT)
                    else:
                        if y*self.ty < 0 or self.tx*self.ty < 0:
                            self.go(LEFT)
                        else:
                            self.go(RIGHT)
            time.sleep(0.01)

        # After the first while loop, and while the robot hasn't left the wall, 
        # this function defines the robot behaviour on how to avoid the obstacle
        while not self.should_leave_wall():
            (front, left, right, backleft, backright) = current_dists.get()
            if front <= DISTANCE:
                if left < right or backleft < backright and stuck_time_2 <= 2:
                    self.go(RIGHT)
                    stuck_time_2 += 0.01
                    self.left_right_stuck = False
                elif left > right or backleft > backright and stuck_time_2 <= 2:
                    self.go(LEFT)
                    stuck_time_2 += 0.01
                else:
                    self.go(MSG_STOP)
                    pass
                time.sleep(0.01)
            elif DISTANCE - 0.2 <= left <= DISTANCE or DISTANCE - 0.2 <= right <= DISTANCE:
                if DISTANCE - 0.2 <= left <= DISTANCE:
                    self.go(MEDIUM_FINE_STRAIGHT)
                    self.right_turn = False
                    self.left_turn = True
                else:
                    self.go(MEDIUM_FINE_STRAIGHT)
                    self.right_turn = True
                    self.left_turn = False
            elif left > DISTANCE - 0.12 and not self.right_turn and stuck_time_3 <= 4:
                self.go(LEFT)
                stuck_time_3 += 0.01
            elif right > DISTANCE - 0.12 and not self.left_turn and stuck_time_3 <= 4:
                self.go(RIGHT)
                stuck_time_3 += 0.01
            else:
                break
            time.sleep(0.01)

    def should_leave_wall(self):
        (x, y, t) = current_location.current_location()
        dir_to_go = current_location.global_to_local(necessary_heading(x, y, self.tx, self.ty))
        at = current_dists.at(dir_to_go)
        if at > 10:
            print ("\nLeaving wall")
            return True
        return False
    

####################################################
            ### Bug 1 STARTS HERE ###
####################################################
    
class Bug1(Bug):
    def __init__(self, algorithm, tx, ty):
        Bug.__init__(self, algorithm, tx, ty)
        self.closest_point = (None, None)
        self.origin = (None, None)
        self.circumnavigated = False

    ## Defines the robot behaviour around walls and/or obstacles
    def follow_wall(self):
        print("\nFollowing wall")
        self.right_turn = False # this two flags define the direction of the previous turn
        self.left_turn = False  # enabling the robot to perfrom the next action correctly
        stuck_time_1 = 0.0
        stuck_time_2 = 0.0
        stuck_time_3 = 0.0

        # This loop executes when the value of DISTANCE is less than the default setting.
        # It determines the direction for the robot to turn based on the relative position of the wall.
        # The loop continues until the DISTANCE is restored to its default value.
        while current_dists.get()[0] <= DISTANCE:
            (x, y, t) = current_location.current_location()
            (front, left, right, backleft, backright) = current_dists.get()
            if (backleft < backright or left < right) and stuck_time_1 <= 2.5:
                if backleft < backright:
                    self.go(RIGHT)
                    stuck_time_1 += 0.01
                else:
                    self.go(LEFT)
                    stuck_time_1 += 0.01
            elif (backleft > backright and left > right) and stuck_time_1 <= 2.5:
                self.go(LEFT)
                stuck_time_1 += 0.01
            else:
                if self.ty <= 0:
                    if  left < right and self.tx < x:
                        self.go(RIGHT)
                        time.sleep(0.2)
                    else:
                        if y*self.ty > 0:
                            self.go(LEFT)
                        else:
                            self.go(RIGHT)
                else:
                    if  left < right and self.tx > x:
                        self.go(RIGHT)
                    else:
                        if y*self.ty > 0:
                            self.go(LEFT)
                        else:
                            self.go(RIGHT)
            time.sleep(0.01)

        # After the first while loop, and while the robot hasn't left the wall, 
        # this function defines the robot behaviour on how to avoid the obstacle
        while not self.should_leave_wall():
            (front, left, right, backleft, backright) = current_dists.get()
            if front <= DISTANCE:
                if left < right or backleft < backright and stuck_time_2 <= 2:
                    self.go(RIGHT)
                    stuck_time_2 += 0.01
                    self.left_right_stuck = False
                elif left > right and backleft > backright and stuck_time_2 <= 2:
                    self.go(LEFT)
                    stuck_time_2 += 0.01
                else:
                    pass
                time.sleep(0.01)
            elif DISTANCE - 0.2 <= left <= DISTANCE or DISTANCE - 0.2 <= right <= DISTANCE:
                if DISTANCE - 0.2 <= left <= DISTANCE:
                    self.go(MEDIUM_FINE_STRAIGHT)
                    self.right_turn = False
                    self.left_turn = True
                else:
                    self.go(MEDIUM_FINE_STRAIGHT)
                    self.right_turn = True
                    self.left_turn = False
            elif left > DISTANCE - 0.12 and not self.right_turn and stuck_time_3 <= 4:
                self.go(LEFT)
                stuck_time_3 += 0.01
            elif right > DISTANCE - 0.12 and not self.left_turn and stuck_time_3 <= 4:
                self.go(RIGHT)
                stuck_time_3 += 0.01
            else:
                if self.right_turn:
                    self.go(RIGHT)
                else:
                    self.go(LEFT)
            time.sleep(0.01)

    def near(self, cx, cy, x, y):
        nearx = x - 0.11 <= cx <= x + 0.11
        neary = y - 0.11 <= cy <= y + 0.11
        return nearx and neary

    def should_leave_wall(self):
        (x, y, t) = current_location.current_location()
        if None in self.closest_point:
            self.origin = (x, y)
            self.closest_point = (x, y)
            self.closest_distance = current_location.distance(self.tx, self.ty)
            self.left_origin_point = False
            return False
        d = current_location.distance(self.tx, self.ty)
        if d <= self.closest_distance:
            print ("\nNew closest point at", x, y)
            self.closest_distance = d
            self.closest_point = (x, y)

        (ox, oy) = self.origin
        if not self.left_origin_point and not self.near(x, y, ox, oy):
            # we have now left the point where we hit the wall
            print ("\n\nLeft original touch point\n")
            self.left_origin_point = True
        elif self.near(x, y, ox, oy) and self.left_origin_point:
            # circumnavigation achieved!
            print ("\nCircumnavigated obstacle")
            self.circumnavigated = True

        (cx, cy) = self.closest_point
        if self.circumnavigated and self.near(x, y, cx, cy):
            self.closest_point = (None, None)
            self.origin = (None, None)
            self.circumnavigated = False
            self.left_origin_point = False
            print ("\nLeaving wall")
            return True
        else:
            return False



####################################################
            ### Bug 2 STARTS HERE ###
####################################################

class Bug2(Bug):
    def __init__(self, algorithm, tx, ty):
        Bug.__init__(self, algorithm, tx, ty)
        self.lh = None
        self.encountered_wall_at = (None, None)

    ## Defines the robot behaviour around walls and/or obstacles
    def follow_wall(self):
        print("\nFollowing wall")
        self.right_turn = False # this two flags define the direction of the previous turn
        self.left_turn = False  # enabling the robot to perfrom the next action correctly
        stuck_time_1 = 0.0
        stuck_time_2 = 0.0
        stuck_time_3 = 0.0
        # This loop executes when the value of DISTANCE is less than the default setting.
        # It determines the direction for the robot to turn based on the relative position of the wall.
        # The loop continues until the DISTANCE is restored to its default value.
        while current_dists.get()[0] <= DISTANCE:
            self.obstacle_time += 0.01
            (x, y, t) = current_location.current_location()
            (front, left, right, backleft, backright) = current_dists.get()
            if (backleft < backright or left < right) and stuck_time_1 <= 3:
                if backleft < backright:
                    # print("Tu sam 2.1.")
                    self.go(RIGHT)
                    stuck_time_1 += 0.01
                else:
                    # print("Tu sam 2.2.")
                    self.go(LEFT)
                    stuck_time_1 += 0.01
            elif (backleft > backright and left > right) and stuck_time_1 <= 3:
                # print("Tu sam 2.3.")
                self.go(LEFT)
                stuck_time_1 += 0.01
            else:
                if self.ty <= 0:
                    if  left < right and self.tx < x:
                        # print("Tu sam 2.4.")
                        self.go(RIGHT)
                    else:
                        if y*self.ty > 0:
                            # print("Tu sam 2.5.")
                            self.go(LEFT)
                        else:
                            # print("Tu sam 2.6.")
                            self.go(RIGHT)
                else:
                    if  left < right and self.tx > x:
                        # print("Tu sam 2.7.")
                        self.go(RIGHT)
                    else:
                        if y*self.ty > 0:
                            # print("Tu sam 2.8.")
                            self.go(LEFT)
                        else:
                            # print("Tu sam 2.9.")
                            self.go(RIGHT)
            time.sleep(0.01)

        # After the first while loop, and while the robot hasn't left the wall, 
        # this function defines the robot behaviour on how to avoid the obstacle
        while not self.should_leave_wall():
            self.obstacle_time += 0.01
            (front, left, right, backleft, backright) = current_dists.get()
            if front <= DISTANCE:
                if left < right or backleft < backright and stuck_time_2 <= 2:
                    # print("Tu sam 3.1.")
                    self.go(RIGHT)
                    stuck_time_2 += 0.01
                elif left > right and backleft > backright and stuck_time_2 <= 2:
                    # print("Tu sam 3.2.")
                    self.go(LEFT)
                    stuck_time_2 += 0.01
                else:
                    break
                time.sleep(0.01)
            elif DISTANCE - 0.2 <= left <= DISTANCE or DISTANCE - 0.2 <= right <= DISTANCE:
                if DISTANCE - 0.2 <= left <= DISTANCE:
                    self.go(MEDIUM_FINE_STRAIGHT)
                    self.right_turn = False
                    self.left_turn = True
                else:
                    self.go(MEDIUM_FINE_STRAIGHT)
                    self.right_turn = True
                    self.left_turn = False
            elif left > DISTANCE - 0.12 and not self.right_turn:
                # print("Tu sam 3.3.")
                self.go(LEFT)
            elif right > DISTANCE - 0.12 and not self.left_turn:
                # print("Tu sam 3.4.")
                self.go(RIGHT)
            else:
                if self.right_turn and stuck_time_3 <= 4:
                    # print("Tu sam 3.5.")
                    self.go(RIGHT)
                    stuck_time_3 += 0.01
                elif self.left_turn and stuck_time_3 <= 4:
                     # print("Tu sam 3.6.")
                     self.go(LEFT)
                     stuck_time_3 += 0.01
                else:
                    break
            time.sleep(0.01)

    def near(self, cx, cy, x, y):
        nearx = x - 0.1 <= cx <= x + 0.1
        neary = y - 0.1 <= cy <= y + 0.1
        return nearx and neary
    
    def face_goal(self):
        while not current_location.facing_point(self.tx, self.ty):
            self.obstacle_time += 0.01
            self.go(RIGHT)

    def bug2_wall(self):
        Bug2.follow_wall()
        self.face_goal()

    def should_leave_wall(self):
        (x, y, _) = current_location.current_location()
        if None in self.encountered_wall_at:
            self.encountered_wall_at = (x, y)
            self.lh = necessary_heading(x, y, self.tx, self.ty)
            return False
        t_angle = necessary_heading(x, y, self.tx, self.ty)
        (ox, oy) = self.encountered_wall_at
        od = math.sqrt((ox-self.tx)**2 + (oy-self.ty)**2) # distance between obstacle hit point and target point
        cd = math.sqrt((x-self.tx)**2 + (y-self.ty)**2)   # distance between current position and target point
        dt = 0.12

        if self.lh - dt <= t_angle <= self.lh + dt and not self.near(x, y, ox, oy):
            if cd < od and self.obstacle_time >= 3:
                print ("\nLeaving wall")
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
        print ("Usage: ros2 run my_simulations bug.py ALGORITHM X Y")
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
    print ("\nStarting algorithm")
    thread = Thread(target=rclpy.spin, args=(bug,))
    thread.start()

    try:
        while rclpy.ok():
            bug.bug_algorithm()
    except KeyboardInterrupt:
        print("\n\nKeyboard interrupt detected. Exiting gracefully.\n")
    finally:
        thread.join()
        rclpy.shutdown()
        bug.destroy_node()

if __name__ == '__main__':
    main()