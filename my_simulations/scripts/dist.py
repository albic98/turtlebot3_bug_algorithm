import threading
import sys


class Dist:
    ## Defining class and initial sensor condition
    ## Initial sensor readings set to 1000 m so that the robot immediately starts looking for the goal
    def __init__(self):
        self.m = threading.Lock()
        self.front = 1000
        self.left = 1000
        self.right = 1000
        self.backleft = 1000
        self.backright = 1000
        self.raw = []

    ## Function which looks for the lowest distance between robot and obstacles in determined ranges of angles 
    def update(self, msg):
        # Function to find the minimum value within a specified range of sensor readings
        def getmin(a, b):
            # Define a lambda function to check if a value is within the range of sensor readings
            in_rng = lambda x: msg.range_min <= x <= msg.range_max
            vsp = list(filter(in_rng, msg.ranges[a:b]))
            # If there are more than 10 valid sensor readings, return the minimum value
            if len(vsp) > 0:
                return min(vsp)
            # If there are fewer than 10 valid sensor readings, return a large value
            else:
                return sys.maxsize

        newfront = min(getmin(330, 360), getmin(0, 30)) # front angles are 0-30 and 330-360
        newleft = getmin(15, 80)                        # left degrees are between  and degrees
        newright = getmin(280, 345)                     # right degrees are between  and  degrees
        new_backleft = getmin(90, 100)                  # backleft degrees are between  and  degrees
        new_backright = getmin(260 ,270)                # backright degrees are between  and  degrees

        # This part of the function just writes the minimum values in the variables which the robot uses
        self.m.acquire()
        self.front = newfront
        self.left = newleft
        self.right = newright
        self.backleft = new_backleft
        self.backright = new_backright
        self
        self.raw = msg
        self.m.release()

    ## This function is used to access the variables in a different script more easily
    def get(self):
        self.m.acquire()
        f = self.front
        l = self.left
        r = self.right
        bl = self.backleft
        br = self.backright
        self.m.release()
        return (f, l, r, bl, br)

    ## Function which converts the angle to an index corresponding to sensor readings
    def angle_to_index(self, angle):
        return int((angle - self.raw.angle_min)/self.raw.angle_increment)

    ## Function which helps us determine if the robot is still near an obstacle, used in Bug 0 algorithm
    def at(self, angle): # angle in radians

        # TODO(exm): copy and paste programming, refactor later
        
        # Function to find the minimum value within a specified range of sensor readings
        def getmin(a, b):
            # Define a lambda function to check if a value is within the range of sensor readings
            in_rng = lambda x: self.raw.range_min <= x <= self.raw.range_max
            vsp = list(filter(in_rng, self.raw.ranges[a:b]))
            # If there are more than 10 valid sensor readings, return the minimum value
            if len(vsp) > 10:
                return min(vsp)
            # If there are fewer than 10 valid sensor readings, return a large value
            else:
                return sys.maxsize

        # Acquire the mutex lock to ensure thread safety
        self.m.acquire()
        i = self.angle_to_index(angle)
        # Define the start index for the range of sensor readings, ensuring it's not less than 0
        start = i - 28
        if start < 0:
            start = 0
        # Define the end index for the range of sensor readings, ensuring it's within the bounds of sensor readings
        end = i + 28
        if end >= len(self.raw.ranges):
            end = len(self.raw.ranges) - 1
        # Call the nested function to find the minimum value within the specified range
        ans = getmin(start, end)
        # Release the mutex lock
        self.m.release()
        # Return the minimum value found within the specified range
        return ans