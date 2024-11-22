import numpy as np
import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches

plt.rcParams.update({'font.size': 64})  
plt.rcParams['axes.labelsize'] = 68  
plt.rcParams['xtick.labelsize'] = 54   
plt.rcParams['ytick.labelsize'] = 54   
plt.rcParams['legend.fontsize'] = 54  

file_path = "/home/weijian/CPDOT/src/formation_planner/traj_result/icra_result/ablation_without_310.yaml"
ef_avg = []
ef_max = []
solve_success = []
h_avg = []
h_std = []
tf = []
cost = []
filter_sort_time = []
solve_ocp_time = []
def read_vectors_from_yaml(file_path):
    result = []
    try:
        with open(file_path, 'r') as file:
            existing_data = yaml.safe_load(file)
        for node in existing_data:
            vector_data = [float(value) for value in node]
            result.append(vector_data)
    except Exception as e:
        print(f"Error: {e}")
    return result

vectors = read_vectors_from_yaml(file_path)

for vector in vectors:
    if ( vector[5] == 1):
        ef_max.append(vector[11])
        ef_avg.append(vector[12])
        h_avg.append(vector[13])
        h_std.append(vector[14])
        tf.append(vector[4])
        cost.append(vector[3])
print('success rate: ', len(ef_max) / 100)
print('ef_max: ', np.average(ef_max), np.std(ef_max))
print('ef_avg: ', np.average(ef_avg), np.std(ef_avg))
print('h_avg: ', np.average(h_avg), np.std(h_avg))
print('h_std: ', np.average(h_std), np.std(h_std))
print('tf: ', np.average(tf), np.std(tf))
print('cost: ', np.average(cost), np.std(cost))

