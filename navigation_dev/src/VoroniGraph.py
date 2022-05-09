#!/usr/bin/env python
# coding: utf-8
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d

points = np.array([[0.5, 0], [1.5, 0], [2.5, 0],
                   [3, 0.5], [3, 1.5], [3, 2.5],
                   [0.5, 3], [1.5, 3], [2.5, 3],
                   [0, 0.5], [0, 1.5], [0, 2.5],
                   [1, 1], [2, 1], [1, 2], [2, 2],
                   [1.5, 1], [2, 1.5], [1.5, 2], [1, 1.5]])
vor = Voronoi(points)

fig = voronoi_plot_2d(vor)
plt.show()

waypoints = vor.vertices

index = 0
index_list = []

for i in range(0, len(waypoints)):
    index = i
    element = waypoints[i]
    if element[0] <= 2 and element[0] >= 1 and element[1] <= 2 and element[1] >= 1:
        index_list.append(index)

# In[168]:


waypoints = np.delete(waypoints, index_list, axis=0)
print(waypoints)
