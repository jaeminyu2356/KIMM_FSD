import matplotlib.pyplot as plt
import numpy as np

# 각도를 라디안으로 변환
angle_degrees = 0.7
angle_radians = np.radians(angle_degrees)

# 원점
origin = np.array([0, 0])

# 첫 번째 벡터 (기준 벡터)
v1 = np.array([1, 0])

# 두 번째 벡터 (0.7도 회전)
v2 = np.array([np.cos(angle_radians), np.sin(angle_radians)])

# 플로팅
plt.figure(figsize=(8, 8))
plt.quiver(*origin, *v1, color='r', scale=10, label='Vector 1')
plt.quiver(*origin, *v2, color='b', scale=10, label='Vector 2 (0.7° rotated)')

plt.xlim(-0.1, 1.2)
plt.ylim(-0.1, 0.2)
plt.axhline(0, color='black',linewidth=0.5)
plt.axvline(0, color='black',linewidth=0.5)
plt.grid(True)
plt.legend()
plt.title("Visualization of 0.7 Degree Angle Difference")
plt.show()

