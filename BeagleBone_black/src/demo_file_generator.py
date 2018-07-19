import numpy as np
import random

def myprint(values):
  for value in values:
    print(value)

total = 1000
std = 2.0

print(total, std)

actual_value = list(map(round, np.random.random(total) * 100))

myprint(actual_value)

random_value = []
for mean in actual_value:
  random_value.append(np.random.normal(mean, std))

myprint(random_value)

