import rospy
from crazyflie_driver.msg import GenericLogData
import collections

INFINITY = 9999999999.9


class DistanceInputData:
    def __init__(self, filter_size):
        self.distance = collections.deque(maxlen=filter_size)
        self.last_update = rospy.Time.now()
        self.closure_rate = 0.0

    def get_distance(self):
        if len(self.distance) <= 0:
            return INFINITY
        return sum(self.distance)/len(self.distance)


class DWM10000DistanceSensor:
    def __init__(self, my_id, timeout, filter_size, distance_offset = 0.0):
        rospy.Subscriber("log_ranges", GenericLogData, self.range_callback)
        # num_drones = rospy.get_param("~dwm1000sensor/num_drones")
        self.my_id = my_id - 1
        self.sensor_data = {}
        self.timeout = timeout
        self.distance_offset = distance_offset
        self.filter_size = filter_size
        # for id in range(num_drones):
        #     if id is self.my_id:
        #         continue
        #     self.sensor_data[id] = DistanceInputData()

    def get_distance(self, id):
        if id not in self.sensor_data:
            print "not in data!!!!!!!!"

        if id not in self.sensor_data or (rospy.Time.now() - self.sensor_data[id].last_update).to_sec() > self.timeout:
            rospy.logwarn("No distance for id: {}".format(id))
            return None
        else:
            return self.sensor_data[id].get_distance()

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
            # if idx is self.my_id:
            #     current_id += 1
                # for idx, current_distance in enumerate(ranges):
            # is unvalid if distance == 0
            valid = (state & (1 << idx)) != 0
            if valid:
                # print current_id
                if current_id not in self.sensor_data:
                    self.sensor_data[current_id] = DistanceInputData(self.filter_size)
                self.sensor_data[current_id].closure_rate = (current_distance + self.distance_offset - self.sensor_data[
                    current_id].get_distance()) / (
                                                                rospy.Time.now() - self.sensor_data[
                                                                    current_id].last_update).to_sec()
                self.sensor_data[current_id].distance.append(current_distance + self.distance_offset)
                # print current_id, self.sensor_data[current_id].distance
                self.sensor_data[current_id].last_update = rospy.Time.now()
                # print "my id: {} cur id: {} range {} valid {}".format(self.my_id, current_id, current_distance, valid)

            current_id += 1
        # print self.get_distance(2)
