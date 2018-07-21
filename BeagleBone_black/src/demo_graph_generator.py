import sys
import matplotlib.pyplot as plt

def old_graph():
    std = 0
    total = 0

    random_values = []
    real_values = []
    filter_values = []

    with open('data.txt') as input_file:
      total, std, process_noise = [float(x) for x in next(input_file).split()]
      for _ in range(0, int(total)):
        real_values.append([float(x) for x in next(input_file).split()])
      for _ in range(0, int(total)):
        random_values.append([float(x) for x in next(input_file).split()])

    with open('filtered_data.txt') as input_file:
      for _ in range(0, int(total)):
        filter_values.append([float(x) for x in next(input_file).split()])

    plt.plot(random_values, label = 'random values')
    plt.plot(real_values, label = 'real_values')
    plt.plot(filter_values, label = 'filtered values')

    plt.legend()

    plt.show()

def graph(device, axis, imu, process_noise):
    # Finding column number
    column = 0
    if device == 'acc':
        column = 1
    if device == 'gyr':
        column = 4

    if axis == 'x':
        column += 0
    if axis == 'y':
        column += 1
    if axis == 'z':
        column += 2

    # Get and plot unfiltered data
    u_data = []
    u_timestamp = []
    unfiltered_file_name = 'imu_' + imu + '_test.txt'
    with open('data/test/' + unfiltered_file_name) as file:
        for x in range(0, 1000):
            next(file)
        for row in file:
            temp = row.split()
            u_timestamp.append(int(temp[0]))
            u_data.append(float(temp[column]))
    plt.plot(u_timestamp[0:100], u_data[0:100], '-o', label = 'unfiltered')
    plt.legend()

    # Get and plot filtered data
    filtered_file_name = 'noise_' + str(process_noise) + '_imu_' + imu + '.txt'
    print(process_noise, filtered_file_name)
    f_data = []
    f_timestamp = []
    with open('data/test/filtered/' + filtered_file_name) as file:
        for row in file:
            temp = row.split()
            f_timestamp.append(int(temp[0]))
            f_data.append(float(temp[column]))
    plt.plot(f_timestamp[0:100], f_data[0:100], '-o', label = 'p = ' + str(process_noise))
    plt.legend()

    plt.axis([f_timestamp[0]-1, f_timestamp[99]+1, 0.3, 0.6])
    plt.show()
    return

# First argument is device either accelerometer (acc) or gyroscope (gyr)
# Second argument is the axis. x, y, or z
# Third argument is the imu number. 0, 1, 2, or 3
# Forth argument is the process noise. A multiple of 0.1 between 0.1 and 5.0 inclusive
graph(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
