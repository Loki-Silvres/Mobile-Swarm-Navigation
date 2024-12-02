import numpy as np

pose1 = np.array([0.42, -0.28, 0])
pose2 = np.array([1.37, 0.26, 0])
pose3 = np.array([2.45, -0.28, 0.15])
depth = np.array([0.26, 0.02, 0.65]) #=0.26, Y=0.02, Z=0.65

print(np.linalg.norm(pose1 - pose2))
print(np.linalg.norm(pose1 - pose3))

print(np.linalg.norm(depth))