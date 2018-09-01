import networkx as nx

class StructuralModel(object):
    '''A directed graph-based representation of a system's structural model

    Author -- Alex Mitrevski
    Contact -- aleksandar.mitrevski@h-brs.de, youssef-mahmoud.youssef@h-brs.de

    '''
    def __init__(self, structural_model):
        '''
        Keyword arguments:
        structural_model -- a networkx.DiGraph instance representing the
                            structural model of a system
        '''
        self.structural_model = structural_model

    def find_independent_sensors(self, sensor, sensor_list):
        '''Returns a list of names of sensors that do not depend
        on the same component as "sensor"

        Keyword arguments:
        sensor -- name of a sensor
        sensor_list -- a list of sensor names

        '''
        independent_sensors = list()
        sensor_ancestors = set(self.structural_model.ancestors(sensor))
        for other_sensor in sensor_list:
            other_sensor_ancestors = set(self.structural_model.ancestors(other_sensor))
            if not sensor_ancestors.intersection(other_sensor_ancestors):
                independent_sensors.append(other_sensor)
        return independent_sensors
