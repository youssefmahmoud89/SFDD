import yaml
import networkx as nx
from networkx.algorithms.dag import ancestors

class ComponentTypeEnum(object):
    '''Constants specifying the component types in a system

    Author -- Alex Mitrevski
    Contact: aleksandar.mitrevski@h-brs.de, youssef-mahmoud.youssef@h-brs.de

    '''
    SENSOR = 'sensor'
    SUBSYSTEM = 'subsystem'

class ComponentParams(object):
    '''Parameters describing a system component

    Author -- Alex Mitrevski
    Contact: aleksandar.mitrevski@h-brs.de, youssef-mahmoud.youssef@h-brs.de

    '''
    def __init__(self):
        ## component name
        self.name = ''

        ## component type
        self.type = ''

        ## parent components
        self.parents = list()

class StructuralModel(object):
    '''A directed graph-based representation of a system's structural model

    Author -- Alex Mitrevski
    Contact -- aleksandar.mitrevski@h-brs.de, youssef-mahmoud.youssef@h-brs.de

    '''
    def __init__(self, model_config_path):
        '''
        Keyword arguments:
        structural_model -- a YAML file describing the structural model of a system
        '''
        model_data = self.__read_model_data(model_config_path)

        ## a networkx.DiGraph instance representing a structural model
        self.structural_model = self.__create_model(model_data)

        ## a list containing the names of the sensors in the system
        self.sensors = list()
        for component in model_data:
            if component.type == ComponentTypeEnum.SENSOR:
                self.sensors.append(component.name)

    def find_independent_sensors(self, sensor, sensor_list):
        '''Returns a list of names of sensors that do not depend
        on the same component as "sensor"

        Keyword arguments:
        sensor -- name of a sensor
        sensor_list -- a list of sensor names

        '''
        independent_sensors = list()
        sensor_ancestors = set(ancestors(self.structural_model, sensor))
        for other_sensor in sensor_list:
            other_sensor_ancestors = set(ancestors(self.structural_model, other_sensor))
            if not sensor_ancestors.intersection(other_sensor_ancestors):
                independent_sensors.append(other_sensor)
        return independent_sensors

    def __read_model_data(self, model_config_path):
        '''Returns a list of "ComponentParams" objects representing
        the system components specified in the config file

        Keyword arguments:
        model_config_path -- path to a YAML file describing the structural model of a system

        '''
        model_config = self.__read_yaml_file(model_config_path)
        components = list()
        for component_name, component_data in model_config['system_components'].items():
            component_params = ComponentParams()
            component_params.name = component_name
            component_params.type = component_data['type']
            component_params.parents = component_data['parents']
            components.append(component_params)
        return components

    def __read_yaml_file(self, file_name):
        '''Returns the data read from the input YAML file

        Keyword arguments:
        file_name -- name of a YAML file

        '''
        file_handle = open(file_name, 'r')
        data = yaml.load(file_handle)
        file_handle.close()
        return data

    def __create_model(self, components):
        '''Returns a networkx.DiGraph object representing the structural model of
        the system described by the parameters in "components"

        Keyword arguments:
        components -- a list of "ComponentParams" objects representing a set of system components

        '''
        model = nx.DiGraph()
        for component in components:
            for parent in component.parents:
                model.add_edge(parent, component.name)
        return model
