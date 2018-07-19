import numpy as np
import random

def myprint(values):
  for value in values:
    print(value)

total = 100
std = 10.0
process_noise = 1.0

print(total, std, process_noise)

# actual_value = list(map(round, np.random.random(total) * 100))
actual_value = [0]
for x in range(1,total):
  actual_value.append(actual_value[len(actual_value) - 1] + round(np.random.random() * 10 - 5))

myprint(actual_value)

random_value = []
for mean in actual_value:
  random_value.append(np.random.normal(mean, std))

myprint(random_value)
