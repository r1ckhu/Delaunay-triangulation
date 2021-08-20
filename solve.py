import numpy as np

A = np.array([[1,1],[3,2]])
B = np.array([1,4])
x = np.linalg.solve(A,B)
print(x)