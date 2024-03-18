import threading
import math
import sys

# Location is used to maintain a single current location of the robot in a
# thread-safe manner such that the callback and readers can all use it without
# issue
class Location:
    def __init__(self):
        self.m = threading.Lock() # global lock b/c easy and not a problem yet
        self.x = 0
        self.y = 0
        self.t = 0
        self.deltaT = 0.1 # how close to angle to be to go

    def update_location(self, x, y, t):
        with self.m:
            self.x = x
            self.y = y
            self.t = t
        # self.m.acquire()
        # self.x = x
        # self.y = y
        # self.t = t
        # self.m.release()

    def current_location(self):
        with self.m:
            x = self.x
            y = self.y
            t = self.t
        # self.m.acquire()
        # x = self.x
        # y = self.y
        # t = self.t
        # self.m.release()
        return x, y, t

    def distance(self, x, y):
        (x0, y0, _) = self.current_location()
        if x0 == None or y0 == None:
            # will be none on the first iteration
            return sys.maxsize
        return math.sqrt((x-x0)**2 + (y-y0)**2)


    def facing_point(self, x, y):
        (cx, cy, current_heading) = self.current_location()
        if None in (cx, cy, current_heading):
            return False
        n = necessary_heading(cx, cy, x, y)

        lower_bound = n - self.deltaT
        upper_bound = n + self.deltaT

        # Handling boundary conditions for angle comparison
        if lower_bound < -math.pi and upper_bound > math.pi:
            # Angle wraps around -π and π
            return -math.pi <= current_heading <= math.pi
        elif lower_bound < -math.pi:
            # Only lower_bound wraps around -π
            return (-math.pi <= current_heading <= upper_bound) or (lower_bound + 2*math.pi <= current_heading <= math.pi)
        elif upper_bound > math.pi:
            # Only upper_bound wraps around π
            return (lower_bound <= current_heading <= math.pi) or (-math.pi <= current_heading <= upper_bound - 2*math.pi)
        else:
            # No wrapping around -π or π
            return lower_bound <= current_heading <= upper_bound

    def faster_left(self, x, y):
        (cx, cy, current_heading) = self.current_location()
        if None in (cx, cy, current_heading):
            return False
        return current_heading - necessary_heading(cx, cy, x, y) < 0
    
    def global_to_local(self, desired_angle):
        (_, _, current_heading) = self.current_location()
        ans = desired_angle - current_heading
        if ans < -math.pi:
            ans += 2 * math.pi
        return ans

# current x, y; target x,y
def necessary_heading(cx, cy, tx, ty):
    return math.atan2(ty-cy, tx-cx)