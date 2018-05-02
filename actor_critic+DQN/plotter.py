import Robotengine as Rengine
import numpy as np
Rot = Rengine.rotate_axis(np.array([[1.], [0.], [0.]]), 3.).dot(Rengine.rotate_axis(np.array([[0.], [0.], [1.]]), 10.))
Rprime = Rot.dot(Rot.dot(Rot.dot(Rot.dot(Rot.dot(Rot)))))
Rprime = np.array([[1., 0.24, 0.52], [0.22, 0.25, -0.65], [0.12, 0.13, 0.33]])
'''print(Rprime)
print(Rengine.Q2rot(Rprime))
print(np.linalg.det(Rprime))
print(np.linalg.det(Rengine.Q2rot(Rprime)))'''