#!/usr/bin/env python
import copy
import math

import g_vector
import point
import polygon
import segment

GEOMETRY_SMALL_NUMBER= 0.00000001
class Ray:
    def __init__(self, i1=None, i2=None, i3=None):
        if isinstance(i1, point.Point):
            self.p0 = point.Point(i1)
            if isinstance(i2, point.Point):
                self.direction = g_vector.GVector(i1, i2)
            if isinstance(i2, g_vector.GVector):
                self.direction = g_vector.GVector(i2)
            else:
                if i3 is None:
                    self.direction = g_vector.GVector(math.cos(i2), math.sin(i2))
                else:
                    self.direction = g_vector.GVector()
                    self.direction = self.direction.set_spherical(i2, i3, 1)
        else:
            self.p0 = point.Point()
            self.direction = g_vector.GVector()

    def invert(self):
        self.direction = self.direction * -1

    def intersect2(self, s, extend_segment_to_line=False):
        angle = self.direction.angle()
        ts = 0
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi

        if abs(abs(angle) - math.pi/2.0) > GEOMETRY_SMALL_NUMBER:
            gamma = self.direction.y / self.direction.x
            ts = ((s.p0.y - self.p0.y) - gamma*(s.p0.x - self.p0.x))/(gamma*(s.p1.x - s.p0.x) - (s.p1.y - s.p0.y))

            if (ts >= 0 and ts <= 1) or extend_segment_to_line:
                sp0sp1 = s.p1 - s.p0
                ps = s.p0 + sp0sp1*ts
                tr = (ps.x - self.p0.x)/self.direction.x
                if tr > 0:
                    return tr, ps, ts
                else:
                    ps = self.p0
                    return polygon.INFINITY, ps, ts
            else:
                ps = self.p0
                return polygon.INFINITY, ps, ts

        else:
            dx = s.p1.x - s.p0.x

            if abs(dx) < GEOMETRY_SMALL_NUMBER:
                if abs(s.p0.x - self.p0.x) < GEOMETRY_SMALL_NUMBER:
                    ps = copy.deepcopy(self.p0)
                    ts = 0
                    return 0, ps, ts
                else:
                    ps = copy.deepcopy(self.p0)
                    return polygon.INFINITY, ps, ts
            else:
                dy = (s.p0.y + (s.p1.y - s.p0.y)*ts) - self.p0.y
                if angle * dy < 0:
                    ps = copy.deepcopy(self.p0)
                    return polygon.INFINITY, ps, ts
                else:
                    sp1sp0 = g_vector.GVector(s.p1 - s.p0)
                    ps = self.p0 + sp1sp0 * ts
                    return abs(dy), ps, ts

    def distance(self, s):
        assert isinstance(s, segment.Segment)
        d, dummy1, dummy2 = self.intersect2(s)
        return d

