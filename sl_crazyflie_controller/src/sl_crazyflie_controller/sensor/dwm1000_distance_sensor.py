import rospy
from crazyflie_driver.msg import GenericLogData

INFINITY = 9999999999.9


class DistanceInputData:
    def __init__(self, num_drones):
        self.distances = [INFINITY] * num_drones
        self.last_update = [rospy.Time.now()] * num_drones
        self.closure_rate = [0.0] * num_drones
        self.num_drones = num_drones


class DWM10000DistanceSensor:
    def __init__(self):
        rospy.Subscriber("log_ranges", GenericLogData, self.range_callback)
        num_drones = rospy.get_param("~dwm1000sensor/num_drones")
        self.sensor_data = DistanceInputData(num_drones=num_drones)

    def range_callback(self, data):
        ranges = data.values[:self.sensor_data.num_drones]

        state = int(data.values[self.sensor_data.num_drones])
        for idx, current_distance in enumerate(ranges):
            # is unvalid if distance == 0
            valid = (state & (1 << idx)) != 0
            if valid:
                self.sensor_data.closure_rate[idx] = (current_distance - self.sensor_data.distances[idx]) / (
                    rospy.Time.now() - self.sensor_data.last_update[idx]).to_sec()
                self.sensor_data.distances[idx] = current_distance
                self.sensor_data.last_update[idx] = rospy.Time.now()
