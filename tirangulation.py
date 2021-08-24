import visualize
import numpy as np
import math

CNT = 10
MAXRANGE = 50

Point = np.dtype([('x', 'i4'), ('y', 'i4'), ('id', 'i4')])
CandidateState = np.dtype(
    [('x', 'i4'), ('y', 'i4'), ('angle', 'float64'), ('id', 'i4')])

edge = np.zeros([CNT, CNT], dtype='i2')
pointSet = np.zeros(0, dtype=Point)


def init(p) -> np.ndarray:
    p = np.empty(0, dtype=Point)
    fo = open("input.in", "r")

    for line in fo.readlines():
        words = line.split()
        p = np.append(p, np.array(
            (words[0], words[1], 0), dtype=Point))
    p.sort(kind='quicksort', order=['x', 'y'])

    for i, point in enumerate(p):
        point['id'] = i
    # print(p)
    return p


def find_base_LR(lb, rb) -> list:

    middle = math.floor((lb + rb) / 2)
    L_points = pointSet.copy()[lb:middle+1]
    R_points = pointSet.copy()[middle+1:rb+1]

    L_points = np.sort(L_points, kind='quicksort', order='y')
    R_points = np.sort(R_points, kind='quicksort', order='y')

    Lmin = L_points[0]
    Rmin = R_points[0]

    for i in range(1, len(R_points)):
        v1 = np.array([Rmin['x']-Lmin['x'], Rmin['y']-Lmin['y']])
        v2 = np.array([R_points[i]['x']-Lmin['x'], R_points[i]['y']-Lmin['y']])
        angle = calAngle(v1, v2)
        if(angle < 0):  # 假如是顺时针旋转
            Rmin = R_points[i]
        if(R_points[i]['y'] > Rmin['y'] and R_points[i]['y'] > Lmin['y']):
            break

    for i in range(1, len(L_points)):
        v1 = np.array([Lmin['x']-Rmin['x'], Lmin['y']-Rmin['y']])
        v2 = np.array([L_points[i]['x']-Rmin['x'], L_points[i]['y']-Rmin['y']])
        angle = calAngle(v1, v2)
        if(angle > 0):  # 假如是逆时针旋转
            Lmin = L_points[i]
        if(L_points[i]['y'] > Rmin['y'] and L_points[i]['y'] > Lmin['y']):
            break

    return [Lmin['id'], Rmin['id']]


def dis(vector1, vector2) -> np.float64:
    return np.sqrt(np.sum(np.square(vector1-vector2)))


def calAngle(v1, v2) -> np.float64:
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

    candArray = np.empty(0, dtype=CandidateState)  # 候选者队列

    side_id = LR_left_id if side == 1 else LR_right_id  # 确认在LR边上与候选点相连的点
    for point_id in range(lb, rb+1):
        # 遍历合格的候选点
        if edge[side_id][point_id] > 0 and point_id >= lb and point_id <= rb and side_id != point_id:
            cx = pointSet[point_id]['x']
            cy = pointSet[point_id]['y']
            candVector = np.array([cx - pointSet[side_id]['x'],
                                   cy - pointSet[side_id]['y']])
            if side == 1:
                LRvector = np.array(
                    [LR_right['x']-LR_left['x'], LR_right['y']-LR_left['y']])
            elif side == 2:
                LRvector = np.array(
                    [LR_left['x']-LR_right['x'], LR_left['y']-LR_right['y']])

            angle = calAngle(LRvector, candVector)  # 计算角度
            if (side == 1 and angle > 0 and angle < 180) or (side == 2 and angle < 0):
                # 要求左侧候选点为逆时针角度，右侧反之
                candArray = np.append(
                    candArray, np.array((cx, cy, abs(angle), point_id), dtype=CandidateState))
            print(" {} is a possible candidate".format(point_id))

    candArray = np.sort(candArray, kind='quicksort', order='angle')

    if len(candArray) == 1:
        print("Candidate for lb:{} rb:{} is {}".format(
            lb, rb, candArray[0]['id']))
        return candArray[0]['id']
    elif len(candArray) == 0:
        print("No candidate in lb:{} rb:{}".format(lb, rb))
        return -1

    for i in range(0, len(candArray)-1):
        cand = np.array(
            (candArray[i]['x'], candArray[i]['y'], candArray[i]['id']), dtype=Point)
        Nextcand = np.array(
            (candArray[i+1]['x'], candArray[i+1]['y'], candArray[i]['id']), dtype=Point)

        print("Testing {} , the next candidate is {}".format(
            candArray[i]['id'], candArray[i+1]['id']))

        if(isOutCircle(p1=LR_left, p2=LR_right, p3=cand, target=Nextcand)):
            print("Candidate for lb:{} rb:{} is {}".format(
                lb, rb, candArray[i]['id']))
            return candArray[i]['id']
        else:
            print("Candidate:{} failed".format(candArray[i]['id']))
            edge[candArray[i]['id']][side_id] = \
                edge[side_id][candArray[i]['id']] = 0
            print("Delete edge between {} {}".format(
                side_id, candArray[i]['id']))

    print("Candidate for lb:{} rb:{} is {}".format(
        lb, rb, candArray[len(candArray)-1]['id']))
    return candArray[len(candArray)-1]['id']


def merge(lb, rb, side):
    global pointSet
    global edge

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
        LR_pointId_list = find_base_LR(lb, rb)
        LR_left_id = LR_pointId_list[0]
        LR_right_id = LR_pointId_list[1]

        while(True):     # 当两边找到下一点后，继续合并，否则结束

            edge[LR_left_id][LR_right_id] = edge[LR_right_id][LR_left_id] = 3  # 创建RL边
            print("Create edge between {} {}".format(LR_left_id, LR_right_id))

            left_cand_id = find_candidate_id(
                lb, middle, LR_left_id, LR_right_id, side=1)    # 寻找左边候选点
            right_cand_id = find_candidate_id(
                middle+1, rb, LR_left_id, LR_right_id, side=2)  # 寻找右边候选点

            print("left c is {}. right c is {}".format(
                left_cand_id, right_cand_id))

            if(left_cand_id != -1 and right_cand_id != -1):  # 两边都有候选点，选取一个
                if isOutCircle(pointSet[LR_left_id], pointSet[LR_right_id],
                               pointSet[left_cand_id], pointSet[right_cand_id]):
                    # 右候选点不在（左候选点，LR边的两点）组成的三角形的外接圆内
                    LR_left_id = left_cand_id
                    print("LEFT candidate wins")
                else:   # 右候选点不满足，选取左候选点
                    LR_right_id = right_cand_id
                    print("RIGHT candidate wins")
            else:   # 只有一边有候选点
                if(left_cand_id != -1):
                    LR_left_id = left_cand_id
                    print("LEFT candidate wins")
                elif(right_cand_id != -1):
                    LR_right_id = right_cand_id
                    print("RIGHT candidate wins")
                else:
                    break


def main():
    global pointSet
    pointSet = init(pointSet)
    #pointSet = visualize.init(cnt=CNT, maxRange=MAXRANGE)
    print(pointSet)
    visualize.draw_point(pointSet)
    merge(0, CNT-1, side=0)
    for i in range(0, CNT):
        for j in range(0, i):
            if edge[i][j] == 1:
                print("{}---{}:LL".format(i, j))
                visualize.connect(pointSet[i], pointSet[j])
            elif edge[i][j] == 2:
                print("{}---{}:RR".format(i, j))
                visualize.connect(pointSet[i], pointSet[j])
            elif edge[i][j] == 3:
                print("{}---{}:LR".format(i, j))
                visualize.connect(pointSet[i], pointSet[j])
    visualize.draw()

if __name__ == '__main__':
    main()

