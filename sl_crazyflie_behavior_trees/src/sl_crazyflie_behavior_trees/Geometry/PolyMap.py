#!/usr/bin/env python
from math import ceil, floor

import point
import polygon


class PolyMap:
    def __init__(self):
        self.m_n = 0
        self.m_nx = 0
        self.m_ny = 0
        self.m_init_cnt = 0
        self.m_mark_cnt = 0
        self.m_step_size = 0
        self.m_offset = point.Point()
        self.m_map = []

    def init(self, poly, step):
        assert isinstance(poly, polygon.Polygon)
        p_max = point.Point()

        if poly.enclosing_rect(self.m_offset, p_max):
            return False

        self.m_step_size = step
        self.m_nx = int(ceil((p_max.x - self.m_offset.x) / self.m_step_size) + 1)
        self.m_ny = int(ceil((p_max.y - self.m_offset.y) / self.m_step_size) + 1)

        self.m_n = self.m_nx * self.m_ny

        self.m_map = [0] * self.m_nx
        for i in range(self.m_nx):
            self.m_map[i] = [0] * self.m_ny

        self.m_init_cnt = 0
        self.m_mark_cnt = 0

        pt = point.Point()
        for i in range(self.m_nx):
            pt.x = i * self.m_step_size + self.m_offset.x
            for j in range(self.m_ny):
                pt.y = j * self.m_step_size + self.m_offset.y

                if poly.point_in_poly(pt):
                    self.m_map[i][j] = 0
                    self.m_init_cnt += 1
                else:
                    self.m_map[i][j] = 1

        return True

    def mark(self, pt):
        assert isinstance(pt, point.Point)
        i = int(floor((pt.x - self.m_offset.x) / self.m_step_size))
        j = int(floor((pt.y - self.m_offset.y) / self.m_step_size))

        if i >= self.m_nx or i < 0:
            return False

        if j >= self.m_ny or j < 0:
            return False

        if self.m_map[i][j] == 2:
            return True

        self.m_map[i][j] = 2
        self.m_mark_cnt += 1

        return True

    def get_map_step_size(self):
        return self.m_step_size

    def get_coverage(self):
        return float(self.m_mark_cnt) / (self.m_n - self.m_init_cnt)

    def get_map_size_x(self):
        return self.m_nx

    def get_map_size_y(self):
        return self.m_ny


