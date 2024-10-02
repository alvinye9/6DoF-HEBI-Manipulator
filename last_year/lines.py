
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np

def line2D(a, b, n=5, graph=False):
    # creates n points evenly spaced along a line from point a to b
    # in 2 dimensions
    result = np.array([a])
    x_split = (b[0] - a[0]) / n
    y_split = (b[1] - a[1]) / n
    for i in range(n):
        new_point = np.array([[a[0] + x_split * (i+1), a[1] + y_split * (i+1)]])
        result = np.append(result, new_point, axis=0)
    if graph:
        print(result)
        x, y = np.split(result, 2, axis=1)
        fig = plt.figure()
        ax = fig.add_subplot()
        ax.plot(x, y, color='blue', linestyle='dashed', linewidth=2, alpha=0.5)
        ax.scatter(x, y, color='green', marker='o', s=40)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        plt.show()
    return result

def line3D(a, b, n=5, graph=False):
    # creates n points evenly spaced along a line from point a to b
    # in 3 dimensions
    result = np.array([a])
    x_split = (b[0] - a[0]) / n
    y_split = (b[1] - a[1]) / n
    z_split = (b[2] - a[2]) / n
    for i in range(n):
        new_point = np.array([[a[0] + x_split * (i+1), a[1] + y_split * (i+1), a[2] + z_split * (i+1)]])
        result = np.append(result, new_point, axis=0)
    if graph:
        print(result)
        x, y, z = np.split(result, 3, axis=1)
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.plot3D(x, y, z, color='blue', linestyle='dashed', linewidth=2, alpha=0.5)
        ax.scatter(x, y, z, color='green', marker='o', s=40)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        plt.show()
    return result

if __name__ == '__main__':
    p1 = np.array([1.0, 2.0])
    p2 = np.array([3.0, 4.0])
    points2d = line2D(p1, p2, n=5, graph=True)
    p3 = np.array([1.0, 2.0, 3.0])
    p4 = np.array([4.0, 5.0, 6.0])
    points3d = line3D(p3, p4, n=10, graph=True)