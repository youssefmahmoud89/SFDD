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

    def monitor_sensors(self, data, timestamps, window_size=-1):
        '''Returns a list of anomalous sensors, namely sensors for which the following holds:
        1. they exhibit at least one of the patterns that we are looking for and
        2. none of the correlated sensors that belong to independent components
           exhibit the same pattern(s)

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

        # we find which patterns are exhibited by the sensors
        patterns = self.__calculate_patterns(investigated_data, timestamps)

        anomalous_sensors = list()
        for i, sensor in enumerate(self.sensor_names):
            # we skip the current sensor if it does not seem
            # to exhibit any of the patterns we are looking for
            if not np.any(patterns[i]):
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
                    anomalous_sensors.append(sensor)
                    break
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
