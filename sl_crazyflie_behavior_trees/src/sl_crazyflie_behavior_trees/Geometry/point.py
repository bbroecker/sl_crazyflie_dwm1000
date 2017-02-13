#!/usr/bin/env pyhon
import g_vector


class Point:
    def __init__(self, x=None, y=None, z=None ):
        if isinstance(x, Point):
            p = x
            self.x = p.x
            self.y = p.y
            self.z = p.z
        else:
            self.x = 0 if x is None else x
            self.y = 0 if y is None else y
            self.z = 0 if z is None else z

    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        z = self.z + other.z
        return Point(x, y, z)

    def __sub__(self, other):
        x = self.x - other.x
        y = self.y - other.y
        z = self.z - other.z
        if isinstance(other, Point):
            return g_vector.GVector(x=x, y=y, z=z)
        return Point(x, y, z)