import numpy as np

class PatternLibrary(object):
    '''A library of sensor measurement patterns

    Author -- Alex Mitrevski
    Contact -- aleksandar.mitrevski@h-brs.de

    '''
    @staticmethod
    def stuck_at(measurements, threshold=1e-3):
        '''Returns True if all elements in "measurements" are
        close to each other; returns False otherwise

        Keyword arguments:
        measurements -- a numpy array of sensor measurements
        threshold -- threshold used for element equality checking (default 1e-3)

        '''
        prev_x = measurements[0]
        for x in measurements:
            if abs(x - prev_x) > threshold:
                return False
            prev_x = x
        return True

    @staticmethod
    def drift(measurements, timesteps, threshold=1e-3):
        '''Returns True if the elements in "measurements" exhibit a drift, i.e.
        if the slope describing the measurements changes above the given threshold
        and then remains constant; returns False otherwise

        Keyword arguments:
        measurements -- a numpy array of sensor measurements
        timesteps -- a numpy array of timestamps at which the measurements were made
        threshold -- threshold used for element equality checking (default 1e-3)

        '''
        prev_slope = (measurements[1] - measurements[0]) / (timesteps[1] - timesteps[0])
        slope_changed = False
        for i in range(1, len(measurements)):
            slope = (measurements[i] - measurements[i-1]) / (timesteps[i] - timesteps[i-1])
            if abs(slope - prev_slope) > threshold:
                if not slope_changed:
                    slope_changed = True
                else:
                    return False
            prev_slope = slope
        return slope_changed
