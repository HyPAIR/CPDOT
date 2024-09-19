#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations  # Provided by geometry2 in ROS2
import math
# from helper_nodes.metadata_publisher import MetadataPublisher
from copy import deepcopy
from rclpy.exceptions import ROSInterruptException
from rclpy.duration import Duration
import time 
from nav_msgs.msg import Path 
import yaml
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
import numpy as np

class FormationControllerNode(Node):  
    def __init__(self):
        super().__init__('formation_controller')  
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        # self.path_array = [self.ObtainPath('0'), self.ObtainPath('1'), self.ObtainPath('2')]
        self.path_array = []
        self.relative_positions_x = [0]
        self.relative_positions_y = [0]
        self.odom_received_1 = False
        self.odom_received_2 = False
        self.odom_received_3 = False
        self.robot_names = ["/robot1", "/robot2", ""]
        # self.robot_names = ["/robot2"]
        self.mir_poses = [Pose() for i in range(len(self.robot_names))]
        self.current_vel = 0.0
        self.current_omega = 0.0
        self.Kx = 2
        self.Ky = 4
        self.Kphi = 1.4
        self.KP_vel = 1.0
        self.KP_omega = 1.0
        self.control_rate = 100
        self.sleep_duration_sec = 1.0 / self.control_rate 
        self.velocity_limit_lin = 0.31
        self.velocity_limit_ang = 1.9
        self.acceleration_limit_lin = 2.0
        self.acceleration_limit_ang = 2.5
        self.robot_path_publishers = []
        self.robot_twist_publishers = []
        for i, robot_name in enumerate(self.robot_names):
            # self.robot_path_publishers.append(self.create_publisher(Path, f'{robot_name}/robot_path', 1))
            self.robot_twist_publishers.append(self.create_publisher(Twist, f'{robot_name}/cmd_vel', 1))
        
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # self.subscriber = self.create_subscription(Odometry, '/robot1/odom', self.odom_callback, qos_profile)
        # self.create_subscription(Odometry, '/robot1/odom', lambda msg, i=0: self.odom_callback(msg, 0), qos_profile)
        self.create_subscription(Odometry, '/robot1/odom', self.odom_callback1, qos_profile)
        self.create_subscription(Odometry, '/robot2/odom', self.odom_callback2, qos_profile)
        self.create_subscription(Odometry, '/odom', self.odom_callback3, qos_profile)
        # rclpy.sleep_for(Duration(seconds=1.0))
        # time.sleep(1.0) 

    def odom_callback1(self, msg):
        # self.get_logger().info('Odom1 received!')
        self.mir_poses[0] = msg.pose.pose
        phi_act = tf_transformations.euler_from_quaternion([
            self.mir_poses[0].orientation.x,
            self.mir_poses[0].orientation.y,
            self.mir_poses[0].orientation.z,
            self.mir_poses[0].orientation.w
        ])
        # print(self.mir_poses[0].position.x, self.mir_poses[0].position.y, phi_act[2])
        self.odom_received_1 = True

    def odom_callback2(self, msg):
        # self.get_logger().info('Odom2 received!')
        self.mir_poses[1] = msg.pose.pose
        phi_act = tf_transformations.euler_from_quaternion([
            self.mir_poses[1].orientation.x,
            self.mir_poses[1].orientation.y,
            self.mir_poses[1].orientation.z,
            self.mir_poses[1].orientation.w
        ])
        # print(self.mir_poses[1].position.x, self.mir_poses[1].position.y, phi_act[2])
        self.odom_received_2 = True

    def odom_callback3(self, msg):
        # self.get_logger().info('Odom3 received!')
        self.mir_poses[2] = msg.pose.pose
        phi_act = tf_transformations.euler_from_quaternion([
            self.mir_poses[2].orientation.x,
            self.mir_poses[2].orientation.y,
            self.mir_poses[2].orientation.z,
            self.mir_poses[2].orientation.w
        ])
        # print(self.mir_poses[2].position.x, self.mir_poses[2].position.y, phi_act[2])
        self.odom_received_3 = True

    def wait_for_odom(self, executor):
        self.get_logger().info('Waiting for odom...')
        while not (self.odom_received_1 and self.odom_received_2 and self.odom_received_3):
        # while i < 30:
            try:
                executor.spin_once(timeout_sec=0.01)
            except Exception as e:
                self.get_logger().error(f'Error while spinning: {e}')
                break
        path_array_ = [self.ObtainPath('0'), self.ObtainPath('1'), self.ObtainPath('2')]
        for i in range (len(path_array_)):
            angle =  tf_transformations.euler_from_quaternion([
                            self.mir_poses[i].orientation.x,
                            self.mir_poses[i].orientation.y,
                            self.mir_poses[i].orientation.z,
                            self.mir_poses[i].orientation.w
                            ])
            self.path_array.append(self.transform_path(path_array_[i], self.mir_poses[i].position.x, self.mir_poses[i].position.y, angle[2], i))

        self.get_logger().info('Odom wait thread has been instructed to stop.')

    def start_odom_thread(self, executor):
        self.odom_thread = threading.Thread(target=self.wait_for_odom, args=(executor,))
        self.odom_thread.start()

    def ObtainPath(self, index):
        path = []
        with open('/home/weijian/Downloads/traj_icra_complex33' + index + '0.yaml', 'r') as file:
            data = yaml.safe_load(file)
            for key, state in data.items():
                x = state['x']
                y = state['y']
                theta = state['theta']
                vel = state['v']
                ang_vel = state['omega']
                path.append([x, y, theta, vel, ang_vel])
            global_position = np.array([path[0][0], path[0][1]])
            origin = path[0][0:2]  
            initial_theta = path[0][2]  
            rotation_matrix = np.array([[np.cos(-initial_theta), -np.sin(-initial_theta)],
                                        [np.sin(-initial_theta),  np.cos(-initial_theta)]])
            for i in range(len(path)):
                global_position = np.array([path[i][0], path[i][1]])
                global_theta = path[i][2]
                
                translated_position = global_position - origin
                
                local_position = np.dot(rotation_matrix, translated_position)
                local_theta = global_theta - initial_theta  
                
                path[i][0] = local_position[0]
                path[i][1] = local_position[1]
                path[i][2] = local_theta
        return path

    def run(self, executor):
        node = rclpy.create_node('formation_controller_node')
        rate = node.create_rate(self.control_rate)  
        thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        thread.start()
        self.derive_robot_paths()
        self.compute_path_lengths()
        print(len(self.path_array[0]))
        path_index = 1
        path_index_old = 0
        distances = [0.0 for _ in range(len(self.robot_names))]
        current_distances = [0.0 for _ in range(len(self.robot_names))]
        distances_old = [10e10 for _ in range(len(self.robot_names))]  
        target_vels = [0.0 for _ in range(len(self.robot_names))]
        target_points = [[0.0, 0.0] for _ in range(len(self.robot_names))]
        target_angles = [0.0 for _ in range(len(self.robot_names))]
        target_omegas = [0.0 for _ in range(len(self.robot_names))]
        target_poses = [Pose() for _ in range(len(self.robot_names))]
        current_vels = [0.0 for _ in range(len(self.robot_names))]
        current_omegas = [0.0 for _ in range(len(self.robot_names))]
        current_thetas = [0.0 for _ in range(len(self.robot_names))]

        for i in range(len(self.robot_names)):
            target_poses[i].position.x = self.robot_paths_x[i][0]
            target_poses[i].position.y = self.robot_paths_y[i][0]
            q = tf_transformations.quaternion_from_euler(0, 0, self.path_array[i][0][2])
            target_poses[i].orientation.x = q[0]
            target_poses[i].orientation.y = q[1]
            target_poses[i].orientation.z = q[2]
            target_poses[i].orientation.w = q[3]
            current_thetas[i] = self.path_array[i][0][2]

        timestamp_old = self.get_clock().now()  # 替换 rospy.Time.now()
        # print('wait for the odom!')
        # self.wait_for_odom()
        # while not self.odom_received:
        #     # if (self.receive_pose(self.mir_poses)):
        #     #     print("All odom received!")
        #     #     break
        #     rate.sleep()
        #     # rclpy.spin_once(self, timeout_sec=0.1)
        # self.get_logger().info('Proceeding with the next step!')

        # main loop
        pos_set_x_1 = np.array([])
        pos_set_y_1 = np.array([])
        pos_set_theta_1 = np.array([])
        pos_set_x_2 = np.array([])
        pos_set_y_2 = np.array([])
        pos_set_theta_2 = np.array([])
        pos_set_x_3 = np.array([])
        pos_set_y_3 = np.array([])
        pos_set_theta_3 = np.array([])
        while path_index < len(self.path_array[0])-2 and rclpy.ok() :
        # while rclpy.ok() :
            # compute distance to next point
            executor.spin_once(timeout_sec=0.001)
            for i in range(len(self.robot_names)):
                #overall_distances[i] += math.sqrt((self.robot_paths_x[i][path_index] - self.robot_paths_x[i][path_index - 1])**2 + (self.robot_paths_y[i][path_index] - self.robot_paths_y[i][path_index - 1])**2)
                distances[i] = self.path_lengths[i][path_index] - current_distances[i]
                # distances[i] = math.sqrt((self.robot_paths_x[i][path_index] - target_poses[i].position.x)**2 + (self.robot_paths_y[i][path_index] - target_poses[i].position.y)**2)

            # compute target velocity
            for i in range(len(self.robot_names)):
                target_vels[i] = self.KP_vel * distances[i] * self.control_rate
                # target_vels[i] = self.path_array[i][path_index][3]

            # limit target velocity
            vel_scaling_factor = 1.0
            for i in range(len(self.robot_names)):
                if abs(target_vels[i]) > self.velocity_limit_lin:
                    if (self.velocity_limit_lin / abs(target_vels[i]) ) < vel_scaling_factor:
                        vel_scaling_factor =  self.velocity_limit_lin / abs(target_vels[i])

            # apply velocity scaling factor
            for i in range(len(self.robot_names)):
                target_vels[i] = target_vels[i] * vel_scaling_factor

            # limit target acceleration
            vel_scaling_factor = 1.0
            for i in range(len(self.robot_names)):
                acc_target = abs(target_vels[i] - current_vels[i]) 
                if acc_target > self.acceleration_limit_lin:
                    if (self.acceleration_limit_lin/acc_target) < vel_scaling_factor:
                        vel_scaling_factor = self.acceleration_limit_lin / acc_target

            # apply velocity scaling factor
            for i in range(len(self.robot_names)):
                target_vels[i] = target_vels[i] * vel_scaling_factor
            vel_scaling_factor = 1.0

            # check if next point is reached
            for i in range(len(self.robot_names)):
                if distances[i] <= abs(target_vels[i]) * self.sleep_duration_sec:
                    path_index += 1
                    distances_old = deepcopy(distances)
                    break
                if distances[i] > distances_old[i] and path_index != path_index_old: # if distance is increasing, we have passed the target point and need to move on to the next one
                    path_index += 1
                    break
            distances_old = deepcopy(distances)
            path_index_old = path_index


            # compute next target point
            for i in range(len(self.robot_names)):
                # target_points[i][0] = self.robot_paths_x[i][path_index+2]*0.5 + self.robot_paths_x[i][path_index+1]*0.5
                # target_points[i][1] = self.robot_paths_y[i][path_index+2]*0.5 + self.robot_paths_y[i][path_index+1]*0.5
                target_points[i][0] = self.robot_paths_x[i][path_index]
                target_points[i][1] = self.robot_paths_y[i][path_index]

            # broadcast target point
            # for i in range(len(self.robot_names)):
                # self.target_pose_broadcaster.sendTransform((target_points[i][0], target_points[i][1], 0.0), (0.0, 0.0, 0.0, 1.0), self.get_clock().now(), self.robot_names[i] + '/target_point', 'map')

            # compute angle to target point
            for i in range(len(self.robot_names)):
                if (np.hypot(target_points[i][1] - target_poses[i].position.y, target_points[i][0] - target_poses[i].position.x) < 1e-1):
                    target_angles[i] = 0.0
                else:
                    target_angles[i] = math.atan2(target_points[i][1] - target_poses[i].position.y, target_points[i][0] - target_poses[i].position.x)
                #target_angles[i] = math.atan2(target_points[i][1] - self.robot_paths_y[i][path_index-1], target_points[i][0] - self.robot_paths_x[i][path_index-1])
            # prevent the angle from jumping from -pi to pi
            for i in range(len(self.robot_names)):
                if target_angles[i] > math.pi:
                    target_angles[i] -= 2*math.pi
                elif target_angles[i] < -math.pi:
                    target_angles[i] += 2*math.pi

            # compute angle error
            for i in range(len(self.robot_names)):
                angle_error = target_angles[i] - current_thetas[i]
                if angle_error > math.pi:
                    angle_error -= 2*math.pi
                elif angle_error < -math.pi:
                    angle_error += 2*math.pi
                target_omegas[i] = self.KP_omega * angle_error
                # target_omegas[i] = self.path_array[i][path_index][4]

            # limit angular velocity
            for i in range(len(self.robot_names)):
                vel_target = abs(target_omegas[i]) #/ rate.sleep_dur.to_sec()
                if vel_target  > self.velocity_limit_ang:
                    if (self.velocity_limit_ang / vel_target) < vel_scaling_factor:
                        vel_scaling_factor = self.velocity_limit_ang / vel_target

            # apply velocity scaling factor
            for i in range(len(self.robot_names)):
                target_vels[i] *= vel_scaling_factor
                target_omegas[i] *= vel_scaling_factor
            vel_scaling_factor = 1.0

            # limit angular acceleration
            for i in range(len(self.robot_names)):
                acc_target = abs(target_omegas[i] - self.current_omega) 
                if acc_target > self.acceleration_limit_ang:
                    if (self.acceleration_limit_ang / acc_target) < vel_scaling_factor:
                        vel_scaling_factor = self.acceleration_limit_ang / acc_target

            # apply velocity scaling factor
            for i in range(len(self.robot_names)):
                target_vels[i] *= vel_scaling_factor
                target_omegas[i] *= vel_scaling_factor

            now = self.get_clock().now()  
            dt = now - timestamp_old  
            # compute target pose for each robot
            for i in range(len(self.robot_names)):
                target_poses[i].position.x += target_vels[i] * math.cos(current_thetas[i]) * (dt.nanoseconds / 1e9)
                target_poses[i].position.y += target_vels[i] * math.sin(current_thetas[i]) * (dt.nanoseconds / 1e9)
                #q = transformations.quaternion_from_euler(0.0, 0.0, target_angles[i])
                q = tf_transformations.quaternion_from_euler(0.0, 0.0, current_thetas[i] + target_omegas[i] * (dt.nanoseconds / 1e9))
                target_poses[i].orientation.x = q[0]
                target_poses[i].orientation.y = q[1]
                target_poses[i].orientation.z = q[2]
                target_poses[i].orientation.w = q[3]

            # broadcast target poses
            # for i in range(len(self.robot_names)):
            #     self.target_pose_broadcaster.sendTransform((target_poses[i].position.x, target_poses[i].position.y, 0.0),
            #                  nav_msgs               (target_poses[i].orientation.x, target_poses[i].orientation.y, target_poses[i].orientation.z, target_poses[i].orientation.w),
            #                                 self.get_clock().now(),
            #                                 "target_pose_" + str(i),
            #                                 "map")

            dt = self.get_clock().now() - timestamp_old
            pos_set_x_1 = np.append(pos_set_x_1, self.mir_poses[0].position.x)
            pos_set_y_1 = np.append(pos_set_y_1, self.mir_poses[0].position.y)
            angle =  tf_transformations.euler_from_quaternion([
                            self.mir_poses[0].orientation.x,
                            self.mir_poses[0].orientation.y,
                            self.mir_poses[0].orientation.z,
                            self.mir_poses[0].orientation.w
                            ])
            pos_set_theta_1 = np.append(pos_set_theta_1, angle[2])
            pos_set_x_2 = np.append(pos_set_x_2, self.mir_poses[1].position.x)
            pos_set_y_2 = np.append(pos_set_y_2, self.mir_poses[1].position.y)
            angle =  tf_transformations.euler_from_quaternion([
                            self.mir_poses[1].orientation.x,
                            self.mir_poses[1].orientation.y,
                            self.mir_poses[1].orientation.z,
                            self.mir_poses[1].orientation.w
                            ])
            pos_set_theta_2 = np.append(pos_set_theta_2, angle[2])
            pos_set_x_3 = np.append(pos_set_x_3, self.mir_poses[2].position.x)
            pos_set_y_3 = np.append(pos_set_y_3, self.mir_poses[2].position.y)
            angle =  tf_transformations.euler_from_quaternion([
                            self.mir_poses[2].orientation.x,
                            self.mir_poses[2].orientation.y,
                            self.mir_poses[2].orientation.z,
                            self.mir_poses[2].orientation.w
                            ])
            pos_set_theta_3 = np.append(pos_set_theta_3, angle[2])
            # print("actual position:", self.mir_poses[i].position.x, self.mir_poses[i].position.y)
            # update current velocities
            for i in range(len(self.robot_names)):
                current_vels[i] = target_vels[i]
                current_omegas[i] = target_omegas[i]
                current_thetas[i] += target_omegas[i] * (dt.nanoseconds / 1e9)
                current_distances[i] += target_vels[i] * (dt.nanoseconds / 1e9)
            timestamp_old = self.get_clock().now()

            # if (self.reach_goal(self.mir_poses, self.path_array)) :
            #     for i in range(len(self.robot_names)):
            #         # publish znav_msgsero velocities
            #         target_velocity.linear.x = 0.0
            #         target_velocity.angular.z = 0.0
            #         self.robot_twist_publishers[i].publish(target_velocity)
            # else:
                # compute control law and publish target velocities
            for i in range(len(self.robot_names)):
                target_velocity = Twist()
                target_velocity.linear.x = target_vels[i]
                target_velocity.angular.z = target_omegas[i]
                u_v, v_w = self.cartesian_controller(self.mir_poses[i],target_poses[i],target_vels[i],target_omegas[i],i)
                # if (u_v < 0.001 and v_w < 0.001):
                #     path_index += 1
                #     break
                # publish target velocities
                target_velocity.linear.x = u_v
                target_velocity.angular.z = v_w
                # target_velocity.linear.x = target_vels[i]
                # target_velocity.angular.z = target_omegas[i]
                self.robot_twist_publishers[i].publish(target_velocity)
            rate.sleep()
        data = {
            'x': pos_set_x_1.tolist(),
            'y': pos_set_y_1.tolist(),
            'theta': pos_set_theta_1.tolist()
        }

        with open('/home/weijian/colcon_ws/src/formation_controller/data/positions_actual_1.yaml', 'w') as file:
            yaml.dump(data, file)
        data = {
            'x': pos_set_x_2.tolist(),
            'y': pos_set_y_2.tolist(),
            'theta': pos_set_theta_2.tolist()
        }

        with open('/home/weijian/colcon_ws/src/formation_controller/data/positions_actual_2.yaml', 'w') as file:
            yaml.dump(data, file)
        data = {
            'x': pos_set_x_3.tolist(),
            'y': pos_set_y_3.tolist(),
            'theta': pos_set_theta_3.tolist()
        }

        with open('/home/weijian/colcon_ws/src/formation_controller/data/positions_actual_3.yaml', 'w') as file:
            yaml.dump(data, file)


    def transform_path(self, path, x0, y0, theta0, index):
        pos_set_x = np.array([])
        pos_set_y = np.array([])
        x_start = path[0][0]
        y_start = path[0][1]
        theta_start = path[0][2]
        dx = x0 - x_start
        dy = y0 - y_start
        dtheta = theta0 - theta_start
        
        new_path = []
        
        for i in range(len(path)):
            x_new = path[i][0] + dx
            y_new = path[i][1] + dy
            
            x_rot = (x_new - x0) * np.cos(dtheta) - (y_new - y0) * np.sin(dtheta) + x0
            y_rot = (x_new - x0) * np.sin(dtheta) + (y_new - y0) * np.cos(dtheta) + y0
            
            theta_rot = path[i][2] + dtheta
            # if theta_rot > math.pi:
            #     theta_rot -= 2*math.pi
            # elif theta_rot < -math.pi:
            #     theta_rot += 2*math.pi
            vel = path[i][3]
            vel_omg = path[i][4]
            new_path.append((x_rot, y_rot, theta_rot, vel, vel_omg))
            pos_set_x = np.append(pos_set_x, x_rot)
            pos_set_y = np.append(pos_set_y, y_rot)
        data = {
            'x': pos_set_x.tolist(),
            'y': pos_set_y.tolist()
        }

        with open('/home/weijian/colcon_ws/src/formation_controller/data/positions_origin_' + str(index + 1) + '.yaml', 'w') as file:
            yaml.dump(data, file)
        return new_path

    def receive_pose(self, mir_poses):
        for i in range(len(mir_poses)):
            if (mir_poses[i].position.x == 0.0 and mir_poses[i].position.y == 0.0):
                return False
        return True
        
    def reach_goal(self, mir_poses, path_array):
        error = 0.0
        for i in range(len(mir_poses)):
            error += math.hypot(mir_poses[i].position.x - path_array[i][-1][0], mir_poses[i].position.y - path_array[i][-1][1])
        if (error / len(mir_poses) < 0.5):
            return True
        else:
            return False

    def cartesian_controller(self,actual_pose,target_pose,target_vel,target_angvel,i = 0):
        phi_act = tf_transformations.euler_from_quaternion([
            actual_pose.orientation.x,
            actual_pose.orientation.y,
            actual_pose.orientation.z,
            actual_pose.orientation.w
        ])

        phi_target = tf_transformations.euler_from_quaternion([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        ])

        R = tf_transformations.quaternion_matrix([
            actual_pose.orientation.x,
            actual_pose.orientation.y,
            actual_pose.orientation.z,
            actual_pose.orientation.w
        ])

        e_x = (target_pose.position.x- actual_pose.position.x)
        e_y = (target_pose.position.y - actual_pose.position.y)
        e_local_x = R[0,0]*e_x + R[1,0]*e_y
        e_local_y = R[0,1]*e_x + R[1,1]*e_y
        u_w = target_angvel + target_vel * (self.Ky * e_local_y + self.Kphi * math.sin(phi_target[2] - phi_act[2]))
        u_v = target_vel * math.cos(phi_target[2] - phi_act[2]) + self.Kx * e_local_x

        # publish metadata
        # self.metadata_publisher.publish_controller_metadata(target_pose = target_pose, actual_pose = actual_pose, target_velocity = target_velocity, publish = True,
        # error = [e_local_x,e_local_y,phi_target[2]-phi_act[2]], robot_id = i) 

        return u_v, u_w


    def compute_path_lengths(self):
        # compute path lengths
        self.path_lengths = [[0] for _ in range(len(self.robot_names))]
        for idx in range(0,len(self.robot_names)):
            for i in range(1,len(self.path_array[0])):
                self.path_lengths[idx].append(self.path_lengths[idx][i-1] + math.sqrt((self.robot_paths_x[idx][i]-self.robot_paths_x[idx][i-1])**2 + (self.robot_paths_y[idx][i]-self.robot_paths_y[idx][i-1])**2))


    def derive_robot_paths(self):
        # derive robot paths from path_array
        self.robot_paths_x = [ [] for i in range(len(self.robot_names))]
        self.robot_paths_y = [ [] for i in range(len(self.robot_names))]
        self.robot_paths_theta = [ [] for i in range(len(self.robot_names))]
        for idx in range(0,len(self.robot_names)):
            for i in range(len(self.path_array[0])):
                self.robot_paths_x[idx].append(self.path_array[idx][i][0])
                self.robot_paths_y[idx].append(self.path_array[idx][i][1])
                self.robot_paths_theta[idx].append(self.path_array[idx][i][2])

        # self.publish_robot_paths()

    def publish_robot_paths(self):
        # publish robot paths
        robot_path = Path()
        robot_path.header.frame_id = "map"
        robot_path.header.stamp = self.get_clock().now()
        for i in range(len(self.robot_names)):
            robot_path = Path()
            robot_path.header.frame_id = "map"
            robot_path.header.stamp = self.get_clock().now()
            for j in range(len(self.robot_paths_x[i])):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now()
                pose.pose.position.x = self.robot_paths_x[i][j]
                pose.pose.position.y = self.robot_paths_y[i][j]
                pose.pose.position.z = 0.0
                q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.robot_paths_theta[i][j])
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                robot_path.poses.append(pose)
            self.robot_path_publishers[i].publish(robot_path)
            # rclpy.sleep_for(Duration(seconds=0.5))
            time.sleep(0.5)

def main(args=None):
    print("Starting formation_controller_node")  
    rclpy.init(args=args)
    node = FormationControllerNode()
    
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.wait_for_odom(executor)  
        # node.start_odom_thread(executor)
        node.run(executor)  
        executor.spin()  
    except ROSInterruptException:
        node.get_logger().info('Shutting down node due to interrupt.')
    finally:
        if node:
            node.destroy_node()  
        rclpy.shutdown() 



if __name__ == "__main__":
    # try:
    #     rospy.init_node('formation_controller')
    #     rospy.loginfo('formation_controller node started')
    #     exe = Formation_controller_node()
    #     exe.run()
    # except rospy.ROSInterruptException:
    #     pass
    # rclpy.init(args=args)  # 初始化ROS2节点
    main()