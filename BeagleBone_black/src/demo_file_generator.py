import numpy as np
import random

import csv
import sys

def myprint(values):
  for value in values:
    print(value)

def generateData():
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

def csvToData():
    with open(sys.argv[1], 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            #print(row)
            str = "";
            for i in range(0,8):
                str += row[i] + " "
            print(str)

csvToData()
