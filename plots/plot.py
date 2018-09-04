import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt('/Users/YousefMahmoud/SFDD/test_data/22_10_2016__14_46_08.log',delimiter = ';',skiprows=1)

timestep = data[:,0] - data[15,0]
plt.plot(timestep[15:],data[15:,20])


#for i in range(0,len(timestep)):
 #   print(timestep[i])
#print(timestep)
plt.show()