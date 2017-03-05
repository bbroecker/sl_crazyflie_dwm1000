#!/usr/bin/env python
import copy

import math

from g_vector import GVector
from point import Point


class Segment:
    def __init__(self, start = None, end=None):
        if start is None and end is None:
            self.p0 = Point()
            self.p1 = Point()
        else:
            self.p0 = Point(start)
            self.p1 = Point(end)

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
            return (p - self.p1).norm()

        b = c1 / c2

        vb = GVector(b * v)

        pb = self.p0 + vb
        return (p - pb).norm()


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

    def fast_distance2D(self, p):
        assert isinstance(p, Point)
        p.z = 0.0

        v = [self.p1.x - self.p0.x, self.p1.y - self.p0.y]
        # v = GVector(self.p1 - self.p0)
        #w = p - self.p0
        w = [p.x - self.p0.x, p.y - self.p0.y]

        c1 = v[0]*w[0] + v[1]*w[1]
        a1 = (p.x - self.p0.x)
        a2 = (p.y - self.p0.y)
        if c1 <= 0:
            return math.sqrt(a1*a1 + a2*a2)

        c2 = v[0]*v[0] + v[1]*v[1]
        a1 = (p.x - self.p1.x)
        a2 = (p.y - self.p1.y)
        if c2 <= c1:
            return math.sqrt(a1*a1 + a2*a2)
            # return (p-self.p1).norm()

        b = c1 / c2

        # tmp = v *b
        # vb = GVector(v[0]*b, v[1]*b, 0)
        vb = [v[0]*b, v[1]*b]

        # pb = Point(self.p0 + vb)
        pb = [self.p0.x + vb[0], self.p0.y + vb[1]]
        # return (p - pb).norm()
        a1 = (p.x - pb[0])
        a2 = (p.y - pb[1])
        return math.sqrt(a1*a1 + a2*a2)

    def intersects(self, s):

        a = Point(self.p0)
        b = Point(self.p1)
        c = Point(s.p0)
        d = Point(s.p1)

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
