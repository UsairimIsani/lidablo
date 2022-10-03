#!/usr/bin/env python
# coding: utf-8

# # Creating a Grid Map from Lidar Data
#
# We will be creating a grid map from lidar data.
#
# The data has been recorded from one of the meeting rooms in the 4th floor of Agora.
#
# ## Data file
#
# The data has been recorded to the `AgoraRoom.csv` file.
# The file has two columns. Each row represents one lidar reading (one direction).
#
# The first column gives angle (0 to 360) and the second column gives the distance measured at that angle (in mm).
#
# ## Polar graph
#
# The first thing we will do is to make a polar graph and see if we can see the shape of the room.
#
# **Note**: in the line when we load the data with `np.genfromtxt`, we only take the values from `100` to `820`. This is approximately 2 scans (720 points). The file actually contains many consecutive scans and you can change this range if you want to.
#


import numpy as np
import matplotlib.pyplot as plt

from math import pi

#
# We load the data from a .csv file (you can open it from the file list in the left to see its content)
#
data = np.genfromtxt('./tip/AgoraRoom.csv', delimiter=',')[100:820]

# First, let's plot using polar coordinates the data in the file
plt.figure("Polar Plot")
plt.polar(pi*data[:, 0]/180, data[:, 1], '.')
plt.show()


#
#   Convert to cartesian
#

angles = np.array(pi*data[:, 0]/180)
distances = np.array(data[:, 1])

xs = distances*np.cos(angles)
ys = distances*np.sin(angles)

plt.plot(xs, ys, '.')
plt.show()
# exit()

#
# Now let's create a grid map. The following code initializes a matrix of all zeros, and then we assign some `1` values to some sample positions. Delete those lines and add one value for each of the positions that the lidar is able to see (only the occupied cells for now, we will worry about free cells in task 6).
#
# We will use the following notation:
# - `0` means unknown
# - `1` means free cell
# - `2` means occupied cell
#
# The map will be a numpy matrix of 80 by 80. We consider that the lidar is in the position `[40,40]` (in the middle of the map), and each cell is 10cm by 10cm (therefore the whole map is 8m by 8m).
#
# **Note:** in this part, you only have to set points with value `2` (the occupied cells detected by the lidar).


room_map = np.zeros([80, 80])

room_map[40, 40] = 2  # This is the lidar position in the center

# Let's add some sample points
room_map[50:69, 50:69] = 1
room_map[50:70, 69] = 2

# Show map (as a matrix)
plt.matshow(room_map)
plt.show()

#
#  The following `bresenham_points` function implements Bresenham's line algorithm. Note that this function includes both the first and last point (the two given to the function) in the output.
#
#  Your task now is to complete the previous map (the `room_map` matrix) marking with a `1` all the cells between the lidar position and the positions of the obstacles/walls detected by the lidar.
#
#  You can change the values in the example before writing your own code to see how it changes.
#
#  Your task is to do the folllowing:
#  - Change the `bresenham_points` function so that the first and last points are **NOT** included in the `point_list` output.
#  - Fill the `room_map` matrix with all the lidar measurements. Do you see any empty spaces that should not be there or everything looks good?
#  - Change the colormap in the `matshow` function, through the `cmap=XXX` argment. You can look at existing colormaps in [https://matplotlib.org/3.1.0/tutorials/colors/colormaps.html](https://matplotlib.org/3.1.0/tutorials/colors/colormaps.html). You can also create your own if you want.


def bresenham_points(p0, p1):

    point_list = []  # We will fill this list with all points in between p0 and p1

    x0, y0 = p0[0], p0[1]
    x1, y1 = p1[0], p1[1]

    dx = abs(x1-x0)
    dy = abs(y1-y0)
    if x0 < x1:
        sx = 1
    else:
        sx = -1

    if y0 < y1:
        sy = 1
    else:
        sy = -1

    err = dx-dy

    while True:
        #print("{}, {}".format(x0, y0))
        point_list.append([x0, y0])
        if x0 == x1 and y0 == y1:
            break  # This means we have finished, so we break the loop

        e2 = 2*err
        if e2 > -dy:
            # overshot in the y direction
            err = err - dy
            x0 = x0 + sx
        if e2 < dx:
            # overshot in the x direction
            err = err + dx
            y0 = y0 + sy

    return point_list


#
#  This is an example that shows the points between [40,40] (the lidar position) and two points: [10,30] and [10,31] (these two points emulate the lidar readings)
#
# NOTE: we can do + because they are lists, this is different with numpy arrays !!!!
test_points = bresenham_points(
    [40, 40], [10, 30]) + bresenham_points([40, 40], [10, 31])
print("Points in the line: {}".format(test_points))

# Now let's put these test points in the previous matrix (remember to delete this line later !!!)
for point in test_points:
    room_map[point[0]][point[1]] = 1

plt.matshow(room_map, cmap="RdPu")
plt.show()
