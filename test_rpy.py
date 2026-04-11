import math
import numpy as np
from scipy.spatial.transform import Rotation as R

r1 = R.from_euler('xyz', [0, 0.2094, 0], degrees=False).as_matrix()
r2 = R.from_euler('xyz', [-math.pi/2, 0, -math.pi/2], degrees=False).as_matrix()

print("Scipy R_base_cam:\n", r1)
print("Scipy R_cam_opt:\n", r2)
print("Scipy composed:\n", r1 @ r2)

from scipy.spatial.transform import Rotation
r = Rotation.from_matrix(r1 @ r2)
print("Scipy q(x,y,z,w):", r.as_quat())

