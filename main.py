import sys
import numpy as np

from sfdd.structural_model import StructuralModel
from sfdd.sfdd import SFDD

if __name__ == '__main__':
    structural_model_file = sys.argv[1]
    data_file = sys.argv[2]
    window_size = int(sys.argv[3])

    structural_model = StructuralModel(structural_model_file)
    sfdd_manager = SFDD(structural_model.sensors, structural_model, 0.8, 2)

    data = np.loadtxt(data_file, skiprows=1, delimiter=';')
    start_time = data[0, 0]
    for i in range(data.shape[0]):
        data[i, 0] = data[i, 0] - start_time

    measurement_cols = np.array([1,2,3,4,8,9,10,19,20,21,22,23,24,25,26,27,28,29,30,35,36])
    for i in range(data.shape[0] - window_size):
        sensors = sfdd_manager.monitor_sensors(data[i:i+window_size, measurement_cols], data[i:i+window_size, 0])
        print(sensors)
