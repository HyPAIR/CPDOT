import numpy as np

def transform_path(path, x0, y0, theta0):
    """
    将路径的起点变换为 (x0, y0, theta0)，并对其余路径点进行相应变换
    :param path: 原始路径，格式为 [(x, y, theta), ...]
    :param x0: 目标起点的 x 坐标
    :param y0: 目标起点的 y 坐标
    :param theta0: 目标起点的 theta 角度
    :return: 转换后的路径
    """
    x_start, y_start, theta_start = path[0]
    
    dx = x0 - x_start
    dy = y0 - y_start
    dtheta = theta0 - theta_start
    
    new_path = []
    
    for x, y, theta in path:
        x_new = x + dx
        y_new = y + dy
        
        x_rot = (x_new - x0) * np.cos(dtheta) - (y_new - y0) * np.sin(dtheta) + x0
        y_rot = (x_new - x0) * np.sin(dtheta) + (y_new - y0) * np.cos(dtheta) + y0
        
        theta_rot = theta + dtheta
        
        new_path.append((x_rot, y_rot, theta_rot))
    
    return new_path

path = [(1, 2, 0.5), (2, 3, 0.6), (3, 4, 0.7)]
x0, y0, theta0 = 10, 10, 1.0

new_path = transform_path(path, x0, y0, theta0)
print(new_path)
