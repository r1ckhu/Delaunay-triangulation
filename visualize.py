import matplotlib.pyplot as plt
import numpy as np
import random
Point = np.dtype([('x', 'i4'), ('y', 'i4'), ('id', 'i4')])
fig = plt.figure()
ax = fig.add_subplot(111)


def init(cnt,maxRange) -> np.ndarray:
    global fig
    global ax
    pointSet = np.empty(0, dtype=Point)
    i = 1
    while(i <= cnt):
        x = random.randint(0, maxRange)
        y = random.randint(0, maxRange)
        newPoint = np.array((x, y, 0), dtype=Point)
        repeat_ = False
        for point in pointSet:
            if point['x'] == x \
                    and point['y'] == y:
                repeat_ = True
        if not repeat_:
            pointSet = np.append(pointSet, newPoint)
            i += 1

    # array_x = pointSet['x']
    # array_y = pointSet['y']
    # ax.plot(array_x, array_y, "o")
    draw_points_in(pointSet)

    pointSet.sort(kind='quicksort', order=['x', 'y'])
    for i, point in enumerate(pointSet):
        point['id'] = i
        
    return pointSet


def draw_line_between(point1, point2):
    global ax
    ArrX = [point1['x'], point2['x']]
    ArrY = [point1['y'], point2['y']]
    ax.plot(ArrX, ArrY)


def draw_points_in(pointSet):
    global ax
    ArrX = pointSet['x']
    ArrY = pointSet['y']
    ax.plot(ArrX, ArrY, "o")


def drawfig():
    global fig
    fig.show()
    plt.pause(0)
