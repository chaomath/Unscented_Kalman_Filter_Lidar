import numpy as np
import matplotlib.pyplot as plt

datafile = open('../build/output.txt', 'r')
lines = datafile.readlines()
line_num = 0
data_list = []
for line in lines:
    if line_num != 0:
        data_list.append([float(x.strip()) for x in line.split()])
    line_num += 1
data_array = np.array(data_list)

x_u = data_array[:, 1]    #ukf x
y_u = data_array[:, 2]    #ukf y
x_m = data_array[:, 7]    #measurement x
y_m = data_array[:, 8]    #measurement y
x_g = data_array[:, 9]    #ground_true x
y_g = data_array[:, 10]   #ground_true y

plt.plot(x_m, y_m,'.', label='measurement')
# plt.plot(x_g, y_g,'.', label='ground_truth')
plt.plot(x_u, y_u, '.-', label='ukf')
plt.xlabel('x(m)')
plt.ylabel('y(m)')
plt.title('UKF process Lidar Data')
plt.legend()
plt.show()
