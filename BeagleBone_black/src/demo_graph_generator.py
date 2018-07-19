import matplotlib.pyplot as plt

std = 0
total = 0

random_values = []
real_values = []
filter_values = []

with open('data.txt') as input_file:
  total, std = [float(x) for x in next(input_file).split()]
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
