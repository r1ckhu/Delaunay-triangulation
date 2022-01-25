# tirangulation.py

## init(p) -> np.ndarray
读取`input.in`里的数据，并储存在`pointSet`中。同时对`pointSet`按照x坐标升序排序（当x坐标相同时，按y升序排列）。返回排序之后的`pointSet`

## find_base_LR(lb, rb) -> list
寻找左右点集的第一条LR边，返回一个列表。列表中的元素分别是LR边左右两点的编号

## calDis(array1, array2) -> np.float64
`array1`与`array2`分别储存两点的x，y坐标。返回两点的距离（最多保留两位小数）

## calAngle(v1, v2) -> np.float64
计算向量`v1`旋转到向量`v2`的夹角，返回值范围(-180,180]，最多保留两位小数，正值为逆时针，负值为顺时针

## isOutCircle(p1, p2, p3, target) -> bool
判断`target`点是否在由点`p1`,`p2`,`p3`三点构成的圆内部

## find_candidate_id(lb, rb, LR_left_id, LR_right_id, side) -> int
寻找在[lb,rb]部分的候选点，`side=1`是为左边，`side=2`时为右边

## merge(lb, rb, side)
合并[lb,rb]之间的点

## main()
程序开始的函数
假如`pointSet = init(pointSet)`，那么会从`input.in`内读取数据
假如`pointSet = visualize.init(cnt=CNT, maxRange=MAXRANGE)`，那么会从`visualize.py`得到一组随机数据
  

---
# visualize.py
## init(cnt,maxRange) -> np.ndarray
随机生成cnt个整数点
## connect(point1, point2)
绘制`point1`与`point2`的直线
## draw_point(pointSet)
绘制`pointSet`里所有的点
## draw()
绘制整个图表