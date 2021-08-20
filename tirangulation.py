import visualize
import numpy as np
import math
Point = np.dtype([('x', 'i4'), ('y', 'i4')])
CandidateState = np.dtype(
    [('x', 'i4'), ('y', 'i4'), ('angle', 'float64'), ('id', 'i4')])

edge = np.zeros([11, 11], dtype='i2')
pointSet = np.zeros(0, dtype=Point)


def init(p):
    p = np.append(p, np.array(
        (0, 0), dtype=Point))
    fo = open("input.in", "r")
    for line in fo.readlines():
        words = line.split()
        p = np.append(p, np.array(
            (words[0], words[1]), dtype=Point))
    p.sort(kind='quicksort', order=['x', 'y'])
    # print(p)
    return p


def find_min_y(lb, rb):
    global pointSet
    min_id = 0
    min_y = np.Inf
    for i in range(lb, rb+1):
        if(pointSet[i]['y'] < min_y):
            min_id = i
            min_y = pointSet[i]['y']
    print("min_id for lb:{}, rb:{} is {}".format(lb, rb, min_id))
    return min_id


# def calPara(point1_id, point2_id):
#     A = pointSet[point2_id]['y']-pointSet[point1_id]['y']  # Y2 - Y1
#     B = pointSet[point1_id]['x']-pointSet[point2_id]['x']  # X1 - X2
#     C = pointSet[point2_id]['x'] * pointSet[point1_id]['y'] - \
#         pointSet[point1_id]['x'] * pointSet[point2_id]['y']  # X2*Y1 - X1*Y2
#     return [A, B, C]


# def isAboveLine(point_id, paraList):
#     global pointSet
#     x = pointSet[point_id]['x']
#     y = pointSet[point_id]['y']
#     if(paraList[1] == 0):
#         return -paraList[0]*x-paraList[2] < 0
#     else:
#         return (y > (-paraList[0]*x-paraList[2])/paraList[1])


def dis(vector1, vector2):
    return np.sqrt(np.sum(np.square(vector1-vector2)))


def calAngle(v1, v2):
    # 计算向量v1旋转到向量v2的夹角，返回值范围(-180,180]，保留两位小数，正值为逆时针，负值为顺时针
    norm = np.linalg.norm(v1) * np.linalg.norm(v2)
    rho = np.rad2deg(np.arcsin(np.cross(v1, v2)/norm))
    theta = np.rad2deg(np.arccos(np.dot(v1, v2)/norm))

    if rho < 0:
        return round(-theta, 2)
    else:
        return round(theta, 2)


def isOutCircle(p1, p2, p3, target) -> bool:
    x0 = np.linalg.det(np.array([
        [p1['x']**2+p1['y']**2, p1['y'], 1],
        [p2['x']**2+p2['y']**2, p2['y'], 1],
        [p3['x']**2+p3['y']**2, p3['y'], 1], ]
    )) / (2*np.linalg.det(np.array([
        [p1['x'], p1['y'], 1],
        [p2['x'], p2['y'], 1],
        [p3['x'], p3['y'], 1]]
    )))
    y0 = np.linalg.det(np.array([
        [p1['x'], p1['x']**2+p1['y']**2, 1],
        [p2['x'], p2['x']**2+p2['y']**2, 1],
        [p3['x'], p3['x']**2+p3['y']**2, 1], ]
    )) / (2*np.linalg.det(np.array([
        [p1['x'], p1['y'], 1],
        [p2['x'], p2['y'], 1],
        [p3['x'], p3['y'], 1]]
    )))

    vp1 = np.array([p1['x'], p1['y']])
    vp2 = np.array([p2['x'], p2['y']])
    vp3 = np.array([p3['x'], p3['y']])
    vpt = np.array([target['x'], target['y']])
    vp0 = np.array([x0, y0])

    a = dis(vp1, vp2)
    b = dis(vp2, vp3)
    c = dis(vp3, vp1)
    p = (a+b+c) / 2
    R = round((a*b*c) / (4 * np.sqrt(p*(p-a)*(p-b)*(p-c))), 3)

    return (round(dis(vpt, vp0), 3) >= R)


def find_candidate_id(lb, rb, LR_left_id, LR_right_id, side) -> int:
    global pointSet
    global edge
    LR_left = pointSet[LR_left_id]
    LR_right = pointSet[LR_right_id]

    print("Finding candidate in {} {}".format(lb, rb))

    candidateArray = np.empty(0, dtype=CandidateState)  # 候选者队列

    side_id = LR_left_id if side == 1 else LR_right_id  # 确认在LR边上与候选点相连的点
    for point_id in range(lb, rb+1):
        # 遍历合格的候选点
        if edge[side_id][point_id] == side and side_id != point_id:
            cx = pointSet[point_id]['x']
            cy = pointSet[point_id]['y']
            candidateVector = np.array([cx - pointSet[side_id]['x'],
                                        cy - pointSet[side_id]['y']])
            if side == 1:
                LRvector = np.array(
                    [LR_right['x']-LR_left['x'], LR_right['y']-LR_left['y']])
            elif side == 2:
                LRvector = np.array(
                    [LR_left['x']-LR_right['x'], LR_left['y']-LR_right['y']])
            angle = calAngle(LRvector, candidateVector)  # 计算角度
            if (side == 1 and angle > 0 and angle < 180) or (side == 2 and angle < 0):
                candidateArray = np.append(
                    candidateArray, np.array((cx, cy, abs(angle), point_id), dtype=CandidateState))
            print(" {} is a possible candidate".format(point_id))

    candidateArray = np.sort(candidateArray, kind='quicksort', order='angle')

    if len(candidateArray) == 1:
        print("Candidate for lb:{} rb:{} is {}".format(
            lb, rb, candidateArray[0]['id']))
        return candidateArray[0]['id']

    for i in range(0, len(candidateArray)):
        candidate = np.array(
            (candidateArray[i]['x'], candidateArray[i]['y']), dtype=Point)
        NextCandidate = np.array(
            (candidateArray[i-1]['x'], candidateArray[i-1]['y']), dtype=Point)

        print("Testing {} , the next candidate is {}".format(
            candidateArray[i]['id'], candidateArray[i-1]['id']))

        if(isOutCircle(p1=LR_left, p2=LR_right, p3=candidate, target=NextCandidate)):
            print("Candidate for lb:{} rb:{} is {}".format(
                lb, rb, candidateArray[i]['id']))
            return candidateArray[i]['id']
        else:
            print("Candiadate:{} failed".format(candidateArray[i]['id']))
            edge[candidateArray[i]['id']][side_id] = \
                edge[side_id][candidateArray[i]['id']] = 0
            print("Delete edge between {} {}".format(
                side_id, candidateArray[i]['id']))
    print("No candidate in lb:{} rb:{}".format(lb, rb))
    return 0


def merge(lb, rb, side):
    global pointSet
    global edgecls

    print("merging {} and {}".format(lb, rb))
    middle = math.floor((lb + rb) / 2)
    if(rb - lb + 1 <= 3):  # 点数少于3，直接相连
        for i in range(lb, rb+1):
            for j in range(lb, rb+1):
                edge[i][j] = side
                print("Create edge between {} {}".format(i, j))
        print("Less than three. Complete!")
        return
    else:
        merge(lb, middle, side=1)
        merge(middle+1, rb, side=2)
        print("BIGGG merging {} and {}".format(lb, rb))
        # 左右子树合并
        LR_left_id = find_min_y(lb, middle)
        LR_right_id = find_min_y(middle+1, rb)  # 两边找到y最小的点

        while(True):     # 当两边找到下一点后，继续合并，否则结束

            edge[LR_left_id][LR_right_id] = edge[LR_right_id][LR_left_id] = 3  # 创建RL边
            print("Create edge between {} {}".format(LR_left_id, LR_right_id))

            left_candidate_id = find_candidate_id(
                lb, middle, LR_left_id, LR_right_id, side=1)    # 寻找左边候选点
            right_candidate_id = find_candidate_id(
                middle+1, rb, LR_left_id, LR_right_id, side=2)  # 寻找右边候选点

            print("left c is {}. right c is {}".format(
                left_candidate_id, right_candidate_id))

            if(left_candidate_id != 0 and right_candidate_id != 0):  # 两边都有候选点，选取一个
                if isOutCircle(pointSet[LR_left_id], pointSet[LR_right_id],
                               pointSet[left_candidate_id], pointSet[right_candidate_id]):
                    # 右候选点不在（左候选点，LR边的两点）组成的三角形的外接圆内
                    LR_left_id = left_candidate_id
                    print("LEFT candidate wins")
                else:   # 右候选点不满足，选取左候选点
                    LR_right_id = right_candidate_id
                    print("RIGHT candidate wins")
            else:   # 只有一边有候选点
                if(left_candidate_id != 0):
                    LR_left_id = left_candidate_id
                    print("LEFT candidate wins")
                elif(right_candidate_id != 0):
                    LR_right_id = right_candidate_id
                    print("RIGHT candidate wins")
                else:
                    break


if __name__ == '__main__':
    pointSet = init(pointSet)
    print(pointSet)
    merge(1, 10, side=0)
    for i in range(1, 11):
        for j in range(1, i):
            if edge[i][j] == 1:
                print("{}---{}:LL".format(i, j))
            elif edge[i][j] == 2:
                print("{}---{}:RR".format(i, j))
            elif edge[i][j] == 3:
                print("{}---{}:LR".format(i, j))
