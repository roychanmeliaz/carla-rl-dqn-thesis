import matplotlib.pyplot as plt
import csv
import itertools
marker = itertools.cycle(('-', '--', '-.', ':')) 

files = [
    'trajectory_50percent_model_5000.csv',
    'trajectory_625percent_model_7900.csv',
    'trajectory_75percent_model_27500.csv',
    'trajectory_100percent_model_11400.csv'
]

for each_file in files:
    filename = open(each_file, 'r')
    file = csv.DictReader(filename)
    x_list = []
    y_list = []    
    for col in file:
        x_list.append(float(col['x']))
        y_list.append(float(col['y']))
    plt.plot(x_list, y_list, next(marker))

plt.legend(['50% throttle','62.5% throttle','75% throttle','100% throttle'], bbox_to_anchor =(0.8, 1.15), ncol = 2)
plt.grid()
plt.axis('equal')
plt.show()
