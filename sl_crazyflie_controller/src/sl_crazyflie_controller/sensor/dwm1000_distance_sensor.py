import rospy
from crazyflie_driver.msg import GenericLogData

INFINITY = 9999999999.9


class DistanceInputData:
    def __init__(self):
        self.distance = INFINITY
        self.last_update = rospy.Time.now()
        self.closure_rate = 0.0


class DWM10000DistanceSensor:
    def __init__(self, my_id, timeout):
        rospy.Subscriber("log_ranges", GenericLogData, self.range_callback)
        # num_drones = rospy.get_param("~dwm1000sensor/num_drones")
        self.my_id = my_id - 1
        self.sensor_data = {}
        self.timeout = timeout
        # for id in range(num_drones):
        #     if id is self.my_id:
        #         continue
        #     self.sensor_data[id] = DistanceInputData()

    def get_distance(self, id):
        if id not in self.sensor_data or (rospy.Time.now() - self.sensor_data[id].last_update).to_sec() > self.timeout:
            rospy.logwarn("No distance for id: {}".format(id))
            return None

    def get_closest_distance_all(self):
        keys = self.sensor_data.keys()
        return self.get_closest(keys)

    def get_closest(self, id_list):
        result = None
        min_distance = INFINITY
        for id in id_list:
            distance = self.get_distance(id)
            if distance is not None and distance < min_distance and (
                rospy.Time.now() - self.sensor_data[id].last_update).to_sec() <= self.timeout:
                min_distance = result = distance
        if result is None:
            rospy.logwarn("No valid distances")
        return result

    def range_callback(self, data):
        ranges = data.values[:len(data.values) - 1]

        state = int(data.values[-1])
        current_id = 1
        for idx, current_distance in enumerate(ranges):
            if idx is self.my_id:
                current_id += 1
                # for idx, current_distance in enumerate(ranges):
            # is unvalid if distance == 0
            valid = (state & (1 << idx)) != 0
            if valid:
                if current_id not in self.sensor_data:
                    self.sensor_data[current_id] = DistanceInputData()
                self.sensor_data[current_id].closure_rate = (current_distance - self.sensor_data[
                    current_id].distance) / (
                                                                rospy.Time.now() - self.sensor_data[
                                                                    current_id].last_update).to_sec()
                self.sensor_data[current_id].distance = current_distance
                self.sensor_data[current_id].last_update = rospy.Time.now()
            current_id += 1
