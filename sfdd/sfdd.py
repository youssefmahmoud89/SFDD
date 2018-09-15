import yaml
import numpy as np
from sfdd.correlation import CorrelationLibrary
from sfdd.patterns import PatternLibrary

class SFDD(object):
    '''An interface implementing the sensor-based fault detection method described in

    E. Khalatschi and M. Kalech, "A sensor-based approach for fault detection and diagnosis
    for robotic systems," Autonomous Systems, vol. 42, no. 6, pp. 1231-1248, 2018.

    Author -- Alex Mitrevski, Youssef Mahmoud Youssef
    Contact -- aleksandar.mitrevski@h-brs.de, youssef-mahmoud.youssef@h-brs.de

    '''
    def __init__(self, sensor_names, structural_model, correlation_threshold, pattern_count):
        '''
        sensor_names -- a list of sensor names
        structural_model -- a networkx.DiGraph instance representing a system
        correlation_threshold -- threshold used for checking whether two sensors are correlated
        pattern_count -- number of different data trends to check for
        '''
        ## a list of sensor names
        self.sensor_names = sensor_names

        ## a networkx.DiGraph instance representing the structural model of a system
        self.structural_model = structural_model

        ## a dictionary in which each key is a sensor name and each value
        ## is a set of sensor indices corresponding to the sensors
        ## that do not depend on the same component as the sensor in question
        self.independent_sensors = dict()
        for sensor in self.sensor_names:
            independent_sensors = self.structural_model.find_independent_sensors(sensor, self.sensor_names)
            sensor_indices = set()
            for other_sensor in independent_sensors:
                sensor_indices.add(self.sensor_names.index(other_sensor))
            self.independent_sensors[sensor] = sensor_indices

        ## threshold used for checking whether two sensors are correlated
        self.correlation_threshold = correlation_threshold

        ## number of different data trends to check for
        self.pattern_count = pattern_count

        ### a dictionary in which each key is a sensor name and each value
        ### is a list of names corresponding to the sensors
        ### that are correlated to the sensor specified by the key
        self.correlated_sensors = dict()
        for sensor in self.sensor_names:
            self.correlated_sensors[sensor] = list()

        ### a dictionary in which each key is a sensor name and each value
        ### is a list of indices corresponding to the sensors
        ### that are correlated to the sensor specified by the key
        self.correlated_sensor_indices = dict()
        for sensor in self.sensor_names:
            self.correlated_sensor_indices[sensor] = list()

        ### a dictionary in which each key is a sensor name and each value
        ### is a list of lists [[x], [other-sensor-name], [y]],
        ### where [x] is a pattern exhibited by the sensor specified by the key
        ### and [y] is a pattern exhibited at the same time by the sensor
        ### specified by [other-sensor-name]
        self.sensor_pattern_pairs = dict()

        ### a dictionary in which each key is a sensor name and each value
        ### is a Boolean value specifying whether the sensor is thought to be working
        self.sensor_working = dict()
        for sensor in self.sensor_names:
            self.sensor_working[sensor] = True

    def learn_correlations(self, data, correlation_file_name):
        # we find all pairwise correlations between the sensor measurements
        # and take their absolute values since we are only interested
        # in the magnitude of the correlations, not their signs
        correlations = np.abs(CorrelationLibrary.pearson(data))

        nan_rows, nan_cols = np.where(np.isnan(correlations))
        correlations[nan_rows, nan_cols] = 1.

        for i in range(correlations.shape[0]):
            sensor_name = self.sensor_names[i]
            correlated_sensor_idx = np.where(correlations[i] > self.correlation_threshold)[0]
            for idx in correlated_sensor_idx:
                if i != idx:
                    self.correlated_sensors[sensor_name].append(self.sensor_names[idx])
                    self.correlated_sensor_indices[sensor_name].append(idx)

        with open(correlation_file_name, 'w') as data_stream:
            yaml.dump(self.correlated_sensors, data_stream, default_flow_style=False)

    def find_normal_patterns(self, data, timestamps, pattern_file_name, window_size=-1):
        for sensor in self.sensor_names:
            self.sensor_pattern_pairs[sensor] = list()

        for i in range(data.shape[0]-window_size):
            investigated_data = data[i:i+window_size, :]

            # we find which patterns are exhibited by the sensors
            patterns = self.__calculate_patterns(investigated_data, timestamps)
            for j, sensor in enumerate(self.sensor_names):
                # we skip the current sensor if it does not seem
                # to exhibit any of the patterns we are looking for
                if not np.any(patterns[j]):
                    continue

                correlated_sensor_idx = self.correlated_sensor_indices[sensor]

                # patterns are mutually exclusive, so there can be
                # only one pattern active at a time
                active_pattern_idx = int(np.where(patterns[j])[0][0])

                # we check if any of the patterns exhibited by the current sensor
                # appears together with a pattern exhibited by any of the correlated sensors
                # that depend on independent components; if we find such a pattern,
                # a list [[x], [other-sensor-idx], [y]] is added to
                # self.sensor_pattern_pairs[sensor], where [x] and [y] are the
                # IDs of the patterns exhibited by sensor and [other-sensor-idx] respectively
                for sensor_idx in correlated_sensor_idx:
                    if sensor_idx not in self.independent_sensors[sensor] \
                    or not np.any(patterns[sensor_idx]):
                        continue

                    active_corr_sensor_pattern_idx = int(np.where(patterns[sensor_idx])[0][0])
                    corr_sensor_name = self.sensor_names[sensor_idx]
                    pattern_tuple = [active_pattern_idx,
                                     corr_sensor_name,
                                     active_corr_sensor_pattern_idx]
                    if pattern_tuple not in self.sensor_pattern_pairs[sensor]:
                        self.sensor_pattern_pairs[sensor].append(pattern_tuple)

        with open(pattern_file_name, 'w') as data_stream:
            yaml.dump(self.sensor_pattern_pairs, data_stream, default_flow_style=False)

    def monitor_sensors_extended(self, data, timestamps, window_size=-1):
        '''Returns a list of anomalous sensors, namely sensors for which the following holds:
        1. they exhibit one of the patterns that we are looking for and
        2. the patterns exhibited by the correlated sensors that belong to independent components
           haven't been observed together with the pattern of the investigated sensor
           in a fault-free data set

        Keyword arguments:
        data -- an m x n numpy array in which the columns represent sensors and
                the rows are sensor measurements in m consecutive time steps
        timestamps -- a list of timestamps at which the measurements were taken
        window_size -- number of measurements to take for identifying
                       correlations between sensors (default -1)

        '''
        # we find which patterns are exhibited by the sensors
        patterns = self.__calculate_patterns(data, timestamps)

        anomalous_sensors = list()
        for i, sensor in enumerate(self.sensor_names):
            # we skip the current sensor if it does not seem
            # to exhibit any of the patterns we are looking for
            if not np.any(patterns[i]):
                continue

            correlated_sensor_idx = self.correlated_sensor_indices[sensor]

            # patterns are mutually exclusive, so there can be
            # only one pattern active at a time
            active_pattern_idx = int(np.where(patterns[i])[0][0])

            # we check if any of the patterns exhibited by the current sensor
            # appears together with a pattern exhibited by any of the correlated sensors
            # that depend on independent components; the sensor is considered anomalous
            # if that is not the case
            for j in correlated_sensor_idx:
                if j not in self.independent_sensors[sensor] or not np.any(patterns[j]):
                    continue

                active_corr_sensor_pattern_idx = int(np.where(patterns[j])[0][0])
                corr_sensor_name = self.sensor_names[j]
                pattern_tuple = [active_pattern_idx,
                                 corr_sensor_name,
                                 active_corr_sensor_pattern_idx]
                if pattern_tuple not in self.sensor_pattern_pairs[sensor]:
                    anomalous_sensors.append(sensor)
                    break
        return anomalous_sensors

    def monitor_sensors_basic(self, data, timestamps, window_size=-1):
        '''Returns a list of anomalous sensors, namely sensors for which the following holds:
        1. they exhibit one of the patterns that we are looking for and
        2. none of the correlated sensors that belong to independent components
           exhibit the same pattern

        Keyword arguments:
        data -- an m x n numpy array in which the columns represent sensors and
                the rows are sensor measurements in m consecutive time steps
        timestamps -- a list of timestamps at which the measurements were taken
        window_size -- number of measurements to take for identifying
                       correlations between sensors (default -1)

        '''
        if window_size == -1:
            window_size = int(data.shape[0] / 2)

        correlation_data = data[0:window_size]
        investigated_data = data[window_size:]

        # we find all pairwise correlations between the sensor measurements
        # and take their absolute values since we are only interested
        # in the magnitude of the correlations, not their signs
        correlations = np.abs(CorrelationLibrary.pearson(correlation_data))

        nan_rows, nan_cols = np.where(np.isnan(correlations))
        correlations[nan_rows, nan_cols] = 1.

        # we find which patterns are exhibited by the sensors
        patterns = self.__calculate_patterns(investigated_data, timestamps)

        anomalous_sensors = list()
        for i, sensor in enumerate(self.sensor_names):
            # we skip the current sensor if it does not seem
            # to exhibit any of the patterns we are looking for
            if not np.any(patterns[i]):
                self.sensor_working[sensor] = True
                continue

            correlated_sensor_idx = set(np.where(correlations[i] > self.correlation_threshold)[0])
            active_pattern_indices = np.where(patterns[i])[0]

            # we check if any of the patterns exhibited by the current sensor
            # are also exhibited by any of the correlated sensors that depend on
            # independent components; the sensor is considered anomalous if that is the case
            for pattern_idx in active_pattern_indices:
                # we get the indices of all sensors that exhibit the same pattern
                pattern_sensors = set(np.where(patterns[:, pattern_idx])[0])

                # we find the indices of the correlated sensors that exhibit the same pattern
                correlated_pattern_sensors = set(correlated_sensor_idx).intersection(pattern_sensors)

                # if none of the correlated sensors that exhibit the same pattern
                # depend on other components, the current sensor is considered anomalous
                if not correlated_pattern_sensors.intersection(self.independent_sensors[sensor]):
                    self.sensor_working[sensor] = not self.sensor_working[sensor]
                    break
                else:
                    self.sensor_working[sensor] = True

        for sensor in self.sensor_names:
            if not self.sensor_working[sensor]:
                anomalous_sensors.append(sensor)
        return anomalous_sensors

    def __calculate_patterns(self, data, timestamps):
        '''Returns an n x self.pattern_count Boolean numpy array
        in which each row represents a sensor and the columns represent
        whether the sensor exhibits each individual pattern

        Keyword arguments:
        data -- an m x n numpy array in which the columns represent sensors and
                the rows are sensor measurements in m consecutive time steps
        timestamps -- a list of timestamps at which the measurements were taken

        '''
        patterns = np.zeros((data.shape[1], self.pattern_count), dtype=bool)
        for i, measurements in enumerate(data.T):
            patterns[i, 0] = PatternLibrary.stuck_at(measurements)
            patterns[i, 1] = PatternLibrary.drift(measurements, timestamps)
        return patterns
