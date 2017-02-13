#!/usr/bin/env python
import copy

from g_vector import GVector
from point import Point


class Segment:
    def __init__(self, start = None, end=None):
        if start is None and end is None:
            self.p0 = Point()
            self.p1 = Point()
        else:
            self.p0 = copy.deepcopy(start)
            self.p1 = copy.deepcopy(end)

    def length(self):
        return (self.p1 - self.p0).norm()

    def angle(self):
        return (self.p1 - self.p0).angle()


    def distance(self, p):
        v = GVector(self.p1 - self.p0)
        w = GVector(p - self.p0)

        c1 = w * v
        if c1 <= 0:
            return (p - self.p0).norm()

        c2 = v * v
        if c2 <= c1:
            return (p - self.p1).norm();

        b = c1 / c2

        vb = GVector(b * v)

        pb = self.p0 + vb
        return (p - pb).norm();


    def distance2D(self, p):
        assert isinstance(p, Point)
        p.z = 0.0

        v = GVector(self.p1 - self.p0)
        w = p - self.p0

        c1 = w*v
        if c1 <= 0:
            return (p - self.p0).norm()

        c2 = v*v
        if c2 <= c1:
            return (p-self.p1).norm()

        b = c1 / c2

        tmp = v *b
        vb = GVector(v*b)

        pb = Point(self.p0 + vb)
        return (p - pb).norm()

    def intersects(self, s):

        a = copy.deepcopy(self.p0)
        b = copy.deepcopy(self.p1)
        c = copy.deepcopy(s.p0)
        d = copy.deepcopy(s.p1)

        rn = (a.y - c.y)*(d.x - c.x) - (a.x - c.x)*(d.y - c.y)
        rd = (b.x - a.x)*(d.y - c.y) - (b.y - a.y)*(d.x - c.x)

        sn = (a.y - c.y)*(b.x - a.x) - (a.x - c.x)*(b.y - a.y)
        sd = (b.x - a.x)*(d.y - c.y) - (b.y - a.y)*(d.x - c.x)


        if rd == 0:
            if sd == 0:
                return True
            return False

        r = rn / rd
        s = sn / sd

        if 0 < r < 1 and 0 < s < 1:
            return True

        return False

    def is_left(self, p):
        return ((self.p1.x - self.p0.x) * (p.y - self.p0.y) - (p.x - self.p0.x) * (self.p1.y - self.p0.y)) > 0
