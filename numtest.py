import numpy as np

def calAngle(v1,v2):
    norm = np.linalg.norm(v1) * np.linalg.norm(v2)
    rho = np.rad2deg(np.arcsin(np.cross(v1,v2)/norm))
    theta = np.rad2deg(np.arccos(np.dot(v1,v2)/norm))

    if rho<0:
        return round(-theta,2)
    else:
        return round(theta,2)

v1 = np.array([0,1])
v2 = np.array([0,-1])
angle = calAngle(v1,v2)
print(type(angle))