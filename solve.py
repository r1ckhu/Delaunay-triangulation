import numpy as np
from numpy.linalg.linalg import det

a = np.array([1, 1, 1], dtype='i4')
b = np.array([2, 2, 2], dtype='i4')
c = np.array([3, 3, 3], dtype='i4')
d = np.array([a, b, c])
e =np.array([1,2,3,4])
e = np.flip(e[1:4],axis=0)
print(e)
