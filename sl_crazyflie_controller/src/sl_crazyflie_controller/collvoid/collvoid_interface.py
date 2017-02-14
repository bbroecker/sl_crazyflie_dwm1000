#!/usr/bin/env python
class CollvoidInterface:
    def __init__(self):
        self.current_cf_pose = None

    def update_cf_pose(self, pose):
        self.current_cf_pose = pose

    # calculates the new target velocity based on the collvoid scheme
    # target_Velocity is the current target velocity towards the goal
    def calculate_velocity(self, current_target_velocity):
        raise NotImplementedError("Should have implemented this")

    # tells the main controller if this bahaviour is active
    def is_active(self):
        raise NotImplementedError("Should have implemented this")