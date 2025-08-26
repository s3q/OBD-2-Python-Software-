import time
import matplotlib.pyplot as plt
from collections import deque
import sys


plt.ion()
ax = plt.subplot()
lines = {}
data_history= {}
pids = ["RBM", "New"]

for pid in pids:
    line, = ax.plot([],[], label=pid)
    lines[pid] = line
    data_history[pid] = []


ax.set_xlabel("Time")
ax.set_ylabel("Value")
ax.set_title("Live Sensor Data")
ax.legend()

values = {
    "RBM": [
        1, 2, 3, 4,2 , 5, 6,3, 2,2
    ],
    "New": [
        3, 2, 1, 6,9 , 20, 10,3, 2,1
    ]
}

for pid in pids:
      for v in values[pid]:

        data_history[pid].append(v)
        lines[pid].set_xdata(range(len(data_history[pid])))
        lines[pid].set_ydata(list(data_history[pid]))


        ax.relim()
        ax.autoscale_view()
        plt.pause(1)

input()
