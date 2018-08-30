'''
This class takes half the window matrix including the previous readings
of components (0...n) and finds the correlation between every component
with all the other components in the matrix. 

For each component n, a set of correlated sensors is produced.

'''
import numpy as np


class CorrelationLibrary(object):

    def pearson(input_matrix):

        return np.corrcoef(input_matrix, rowvar=True)

