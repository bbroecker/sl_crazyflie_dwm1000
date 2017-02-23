#!/usr/bin/env python
import copy

from numpy.core.umath import NAN
from sl_crazyflie_behavior_trees.Geometry.ray import Ray

import g_vector
import point
import segment

INFINITY = 99999999.9

class Polygon:
    def __init__(self, other = None):
        if other is not None:
            self.m_locked = other.m_locked
            self.m_min = other.m_min
            self.m_max = other.m_max
            self.m_vertices = copy.deepcopy(other.m_vertices)
        else:
            self.m_locked = False
            self.m_vertices = []

    def add_vertex(self, pt):
        assert isinstance(pt, point.Point)

        if pt.z != 0 or self.m_locked:
            return False

        self.m_vertices.append(pt)
        return True

    def get_vertex_count(self):
        return len(self.m_vertices)

    def lock(self):

        self.m_min = copy.deepcopy(self.m_vertices[0])
        self.m_max = copy.deepcopy(self.m_vertices[0])

        for vert in self.m_vertices:
            if vert.x > self.m_max.x:
                self.m_max.x = vert.x
            if vert.y > self.m_max.y:
                self.m_max.y = vert.y
            if vert.x < self.m_min.x:
                self.m_min.x = vert.x
            if vert.y < self.m_min.y:
                self.m_min.y = vert.y


        self.m_vertices.append(self.m_vertices[0])
        self.m_locked = True

    def clear(self):
        self.m_locked = False
        self.m_vertices = []

    def enclosing_rect(self, min, max):
        assert isinstance(min, point.Point)
        assert isinstance(max, point.Point)
        if not self.m_locked:
            return False

        min = copy.deepcopy(self.m_min)
        max = copy.deepcopy(self.m_max)
        return True

    def point_in_poly(self, p):
        assert isinstance(p, point.Point)
        wn = 0
        # loop through all edges of the polygon
        for i in range(len(self.m_vertices)-1):   # edge from V[i] to  V[i+1]
            if self.m_vertices[i].y <= p.y:         # start y <= P.y
                if self.m_vertices[i+1].y > p.y:      # an upward crossing
                    if self.triangle_area(self.m_vertices[i], self.m_vertices[i+1], p, True) > 0:
                        wn += 1           # have  a valid up intersect
            else:                        # start y > P.y (no test needed)
                if self.m_vertices[i+1].y <= p.y:     # a downward crossing
                    if self.triangle_area(self.m_vertices[i], self.m_vertices[i+1], p, True) < 0:
                        wn -= 1           # have  a valid down intersect

        if wn == 0:
            return False
        return True

    def is_self_intersecting(self):

        intersection_count = 0

        if not self.m_locked:
            self.m_vertices.append(self.m_vertices[0])

        if len(self.m_vertices) > 4:
            for i in range(len(self.m_vertices) - 1):
                p0 = self.m_vertices[i]
                p1 = self.m_vertices[i + 1]
                s0 = segment.Segment(p0, p1)
                for j in range(i+1, len(self.m_vertices) -1 ):
                    p2 = self.m_vertices[j]
                    p3 = self.m_vertices[j+1]
                    s1 = segment.Segment(p2, p3)
                    if s0.intersects(s1):
                        intersection_count += 1

        if not self.m_locked:
            self.m_vertices.pop()

        return intersection_count

    def simplify(self):
        if self.m_locked:
            self.lock()

        if self.is_self_intersecting():
            return True

        vertices = copy.deepcopy(self.m_vertices)
        vertices.pop()

        dummy = Polygon()
        while len(vertices) > 0:

            for i in range(len(dummy.m_vertices) + 1):
                # print "0. {0} {1}".format(len(dummy.m_vertices), i)
                dummy.m_vertices.insert(i, copy.deepcopy(vertices[0]))
                # print "1. {0} {1}".format(len(dummy.m_vertices), i+1)
                if len(vertices) > 1:
                    dummy.m_vertices.insert(i + 1, copy.deepcopy(vertices[1]))
                if not dummy.is_self_intersecting():
                    vertices.pop(0)
                    break
                else:
                    dummy.m_vertices.pop(i)
        dummy.lock()
        self.m_vertices = []
        self.m_vertices = copy.deepcopy(dummy.m_vertices)
        return True


    def min_distance(self, p):
        assert isinstance(p, point.Point)

        dist = g_vector.GVector(self.m_min, self.m_max).norm()

        i = 0
        while i < len(self.m_vertices) - 1:
            p0 = self.m_vertices[i]
            i += 1
            p1 = self.m_vertices[i]
            tmp = segment.Segment(p0, p1).distance2D(p)
            if tmp < dist:
                dist = tmp
        return dist

    def min_distance_idx(self, pt, s_idx=-1):
        assert isinstance(pt, point.Point)
        s = segment.Segment()

        dist = g_vector.GVector(self.m_min, self.m_max).norm()
        segment_counter = 0

        for i in range(0, len(self.m_vertices) - 1, 1):

            p0 = copy.deepcopy(self.m_vertices[i])
            p1 = copy.deepcopy(self.m_vertices[i + 1])
            segment_counter += 1
            tmp = segment.Segment(p0, p1)
            tmp = tmp.distance2D(pt)
            # print "new idx!!!! {0} {1}".format(tmp, dist)
            if tmp < dist:
                dist = tmp
                s.p0 = p0
                s.p1 = p1
                s_idx = segment_counter

        return dist, s, s_idx

    def max_distance(self, pt):
        assert isinstance(pt, point.Point)
        dist = 0
        for i in range(0, len(self.m_vertices)-1, 1):
            p0 = copy.deepcopy(self.m_vertices[i])
            p1 = copy.deepcopy(self.m_vertices[i + 1])
            tmp = segment.Segment(p0, p1).distance2D(pt)
            if tmp > dist:
                dist = tmp
        return True


    def get_area(self, signed_area = False):
        a = 0
        for i in range(len(self.m_vertices) -1):
            a += self.m_vertices[i].x + self.m_vertices[i+1].y - self.m_vertices[i+1].x*self.m_vertices[i].y

        if a < 0 and not signed_area:
            a = -1 * a

        return a / 2.0

    def getCentroid(self):

        if not self.m_locked:
            self.lock()

        if len(self.m_vertices) == 2:
            return self.m_vertices[0]

        if len(self.m_vertices) == 3:
            p = self.m_vertices[0]
            v = g_vector.GVector(self.m_vertices[0], self.m_vertices[1])

            p = p + 0.5 * v

            return p

        cx = 0
        cy = 0
        a = 0

        for i in range(0, len(self.m_vertices) - 1, 2):
            pt = self.m_vertices[i]
            ptp1 = self.m_vertices[i + 1]

            a += pt.x * ptp1.y - ptp1.x * pt.y
            cx += pt.x + ptp1.x * pt.x * ptp1.y - ptp1.x * pt.y
            cy += pt.y + ptp1.y * pt.x * ptp1.y - ptp1.x * pt.y

        a = a / 2;
        cx /= (6 * a);
        cy /= (6 * a);

        return point.Point(cx, cy, 0)


    def scale(self, factor):
        centroid = self.getCentroid()
        max = point.Point(-INFINITY, -INFINITY)
        min = point.Point(INFINITY, INFINITY)

        for i in range(len(self.m_vertices)):

            self.m_vertices[i].x = (self.m_vertices[i].x - centroid.x) * factor + centroid.x
            self.m_vertices[i].y = (self.m_vertices[i].y - centroid.y) * factor + centroid.y

            if self.m_vertices[i].x > max.x:
                max.x = self.m_vertices[i].x
            if self.m_vertices[i].x < min.x:
                min.x = self.m_vertices[i].x
            if self.m_vertices[i].y > max.y:
                max.y = self.m_vertices[i].y
            if self.m_vertices[i].y < min.y:
                min.y = self.m_vertices[i].y

        self.m_min = min
        self.m_max = max

    def triangle_area(self, P0, P1, P2, signed_area):
        if signed_area:
            return ((P1.x - P0.x) * (P2.y - P0.y) - (P2.x - P0.x) * (P1.y - P0.y)) / 2.0
        else:
            return abs((P1.x - P0.x) * (P2.y - P0.y) - (P2.x - P0.x) * (P1.y - P0.y)) / 2.0


    def ray_reflection_vector(self, ray, bounce_segment):
        assert isinstance(ray, Ray)
        assert isinstance(bounce_segment, segment.Segment)

        impact_point = point.Point()
        ts = 0
        dist, impact_point, ts = ray.intersect2(bounce_segment, True)
        if dist < INFINITY:
            l2 = (bounce_segment.p1 - bounce_segment.p0).norm2()
            p0rp0 = (ray.p0 - bounce_segment.p0)
            p0p1 = (bounce_segment.p1 - bounce_segment.p0)

            tpp = (p0rp0 * p0p1) / l2
            pb = ray.p0 + p0p1 * (2*(ts - tpp))

            return pb - impact_point

        return g_vector.GVector(NAN, NAN, NAN)



    def distance(self, ray):
        dist = g_vector.GVector(self.m_min, self.m_max).norm()
        tmp_point = point.Point()
        tmp_ts = 0;
        impact_point = point.Point()
        ts = 0

        for i in range(len(self.m_vertices) - 1):
            s = segment.Segment(self.m_vertices[i], self.m_vertices[i + 1])
            tmp, tmp_point, tmp_ts = ray.intersect2(s)
            if tmp < dist:
                dist = tmp
                impact_point = copy.deepcopy(tmp_point)
                ts = tmp_ts

        return dist, impact_point, ts

