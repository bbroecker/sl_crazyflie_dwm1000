#!/usr/bin/env python
import math

import point


class GVector:
    def __init__(self, x=None, y=None, z=None):
        if isinstance(x, GVector):
            self.x = x.x
            self.y = x.y
            self.z = x.z
        elif isinstance(x, point.Point) and isinstance(y, point.Point):
            self.x = y.x - x.x
            self.y = y.y - x.y
            self.z = y.z - x.z
        elif isinstance(x, point.Point):
            self.x = x.x
            self.y = x.y
            self.z = x.z
        else:
            self.x = 0 if x is None else x
            self.y = 0 if y is None else y
            self.z = 0 if z is None else z


    def set_spherical(self, az, el = None, r = None):
        el = 0 if el is None else el
        r = 1 if r is None else r
        self.x = r * math.sin(az)*math.cos(el)
        self.y = r * math.sin(az) * math.sin(el)
        self.z = r * math.cos(az)

    def set_polar(self, theta, r=None):
        r = 1 if r is None else r
        self.x = r * math.cos(theta)
        self.y = r * math.sin(theta)
        self.z = 0

    def get_spherical(self):
        r = math.sqrt(self.x*self.x + self.y*self.y + self.z*self.z)
        el = math.atan2(self.z, self.r)
        az = math.atan2(self.y, self.x)
        return az, el, r


    def getPolar(self):
        r = math.sqrt(self.x*self.x + self.y*self.y)
        theta = math.atan2(self.y, self.x)
        return theta, r

    def normalize(self):
        N = self.norm()
        self.x /= N
        self.y /= N
        self.z /= N

    def angle(self):
        return math.atan2(self.y, self.x)

    def angles(self,):
        az, el, dummy = self.get_spherical()
        return az, el

    def cross(self, vec):
        return GVector(self.y * vec.z - self.z * vec.y, self.z * vec.x - self.x * vec.z, self.x * vec.y - self.y * vec.x)

    def norm(self):
        return math.sqrt(self.x*self.x + self.y*self.y + self.z*self.z)

    def norm2(self):
        return self.x*self.x + self.y*self.y + self.z*self.z

    def perp2(self, v):
        return self.x * v.y - self.y * v.x

    def __add__(self, other):
        return GVector(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return GVector(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other):
        if isinstance(other, GVector):
            return self.x * other.x + self.y * other.y + self.z * other.z
        else:
            return GVector(self.x * other, self.y * other, self.z * other)

