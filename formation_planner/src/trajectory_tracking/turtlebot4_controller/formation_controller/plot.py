import yaml
import matplotlib.pyplot as plt

# with open('/home/weijian/colcon_ws/src/formation_controller/data/positions_origin.yaml', 'r') as file:
#     data_origin = yaml.safe_load(file)
with open('/home/weijian/colcon_ws/src/formation_controller/data/positions_origin_1.yaml', 'r') as file:
    data_origin_1 = yaml.safe_load(file)
with open('/home/weijian/colcon_ws/src/formation_controller/data/positions_actual_1.yaml', 'r') as file:
    data_actual_1 = yaml.safe_load(file)
with open('/home/weijian/colcon_ws/src/formation_controller/data/positions_origin_2.yaml', 'r') as file:
    data_origin_2 = yaml.safe_load(file)
with open('/home/weijian/colcon_ws/src/formation_controller/data/positions_actual_2.yaml', 'r') as file:
    data_actual_2 = yaml.safe_load(file)
with open('/home/weijian/colcon_ws/src/formation_controller/data/positions_origin_3.yaml', 'r') as file:
    data_origin_3 = yaml.safe_load(file)
with open('/home/weijian/colcon_ws/src/formation_controller/data/positions_actual_3.yaml', 'r') as file:
    data_actual_3 = yaml.safe_load(file)
# with open('/home/weijian/colcon_ws/src/formation_controller/data/positions_actual_3.yaml', 'r') as file:
#     data_actual_3 = yaml.safe_load(file)
pos_set_x_1 = data_origin_1['x']
pos_set_y_1 = data_origin_1['y']
pos_set_x_1_ = data_actual_1['x']
pos_set_y_1_ = data_actual_1['y']
pos_set_x_2 = data_origin_2['x']
pos_set_y_2 = data_origin_2['y']
pos_set_x_2_ = data_actual_2['x']
pos_set_y_2_ = data_actual_2['y']
pos_set_x_3 = data_origin_3['x']
pos_set_y_3 = data_origin_3['y']
pos_set_x_3_ = data_actual_3['x']
pos_set_y_3_ = data_actual_3['y']
# pos_set_x_3 = data_actual_3['x']
# pos_set_y_3 = data_actual_3['y']

fig, axes = plt.subplots(1, 3, figsize=(15, 5))

axes[0].scatter(pos_set_x_1, pos_set_y_1, c='r', s=10, marker='*', label='Reference Trajectory')
axes[0].set_title('ROBOT 1')
axes[0].axis('equal')
axes[0].legend(loc='upper right')  

axes[0].scatter(pos_set_x_1_, pos_set_y_1_, c='g', s=10, marker='*', label='Actual Trajectory')
axes[0].set_title('ROBOT 1')
axes[0].axis('equal')
axes[0].legend(loc='upper right')  

axes[1].scatter(pos_set_x_2, pos_set_y_2, c='r', s=10, marker='*', label='Reference Trajectory')
axes[1].set_title('ROBOT 2')
axes[1].axis('equal')
axes[1].legend(loc='upper right')  

axes[1].scatter(pos_set_x_2_, pos_set_y_2_, c='g', s=10, marker='*', label='Actual Trajectory')
axes[1].set_title('ROBOT 2')
axes[1].axis('equal')
axes[1].legend(loc='upper right')  

axes[2].scatter(pos_set_x_3, pos_set_y_3, c='r', s=10, marker='*', label='Reference Trajectory')
axes[2].set_title('ROBOT 3')
axes[2].axis('equal')
axes[2].legend(loc='upper right')  

axes[2].scatter(pos_set_x_3_, pos_set_y_3_, c='g', s=10, marker='*', label='Actual Trajectory')
axes[2].set_title('ROBOT 3')
axes[2].axis('equal')
axes[2].legend(loc='upper right')  

plt.tight_layout()
plt.show()