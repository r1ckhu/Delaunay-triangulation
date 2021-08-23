import matplotlib.pyplot as plt
import numpy as np
import random

from numpy.core.fromnumeric import repeat
Point = np.dtype([('x', 'i4'), ('y', 'i4')])
fig = plt.figure()
ax = fig.add_subplot(111)


def init():
    global fig
    global ax
    pointSet = np.zeros(0, dtype=Point)
    #pointSet = np.empty(0, dtype=Point)
    pointSet = np.append(pointSet, np.array(
        (0, 0), dtype=Point))
    i = 1
    while(i <= 10):
        x = random.randint(0, 5)
        y = random.randint(0, 5)
        newPoint = np.array((x, y), dtype=Point)
        repeat_ = False
        for point in pointSet:
            if point['x'] == x \
                and point['y']== y:
                repeat_=True
        if not repeat_:
            pointSet = np.append(pointSet, newPoint)
            i += 1

    array_x = pointSet['x']
    array_y = pointSet['y']
    ax.plot(array_x, array_y, "o")
    pointSet.sort(kind='quicksort', order=['x', 'y'])
    print(pointSet)
    return pointSet


def connect(point1, point2):
    global ax
    ArrX = [point1['x'], point2['x']]
    ArrY = [point1['y'], point2['y']]
    ax.plot(ArrX, ArrY)

def draw_point(pointSet):
    global ax
    ArrX = pointSet['x']
    ArrY = pointSet['y']
    ax.plot(ArrX,ArrY,"o")



def draw():
    global fig
    fig.show()
    plt.pause(0)
