import threading
import sys


class Dist:
    def __init__(self):
        self.m = threading.Lock()
        self.front = 1000
        self.left = 1000
        self.right = 1000
        self.backleft = 1000
        self.backright = 1000
        self.raw = []

    def update(self, msg):
        # these magic numbers were acquired from Alan Beadle
        # straight ahead is 540, 40 index range should be enough
        # left chosen to look slightly back to get in front of wall before turning
        def getmin(a, b):
            in_rng = lambda x: msg.range_min <= x <= msg.range_max
            vsp = list(filter(in_rng, msg.ranges[a:b]))
            if len(vsp) > 0:
                return min(vsp)
            else:
                return sys.maxsize

        newfront = min(getmin(330, 360), getmin(0, 30))
        newleft = getmin(30, 90)
        newright = getmin(270, 330)
        new_backleft = getmin(90, 110)
        new_backright = getmin(250 ,270)

        self.m.acquire()
        self.front = newfront
        self.left = newleft
        self.right = newright
        self.backleft = new_backleft
        self.backright = new_backright
        self
        self.raw = msg
        self.m.release()

    def get(self):
        self.m.acquire()
        f = self.front
        l = self.left
        r = self.right
        bl = self.backleft
        br = self.backright
        self.m.release()
        return (f, l, r, bl, br)

    def angle_to_index(self, angle):
        return int((angle - self.raw.angle_min)/self.raw.angle_increment)

    # angle in radians
    def at(self, angle):
        # TODO(exm): copy and paste programming, refactor later
        def getmin(a, b):
            in_rng = lambda x: self.raw.range_min <= x <= self.raw.range_max
            vsp = list(filter(in_rng, self.raw.ranges[a:b]))
            if len(vsp) > 10:
                return min(vsp)
            else:
                return sys.maxsize
            
        self.m.acquire()
        i = self.angle_to_index(angle)
        start = i - 30
        if start < 0:
            start = 0
        end = i + 30
        if end >= len(self.raw.ranges):
            end = len(self.raw.ranges) - 1
        ans = getmin(start, end)
        self.m.release()
        return ans