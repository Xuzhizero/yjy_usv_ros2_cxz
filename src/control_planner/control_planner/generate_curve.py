#! /usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import os

def save_data(y_data, omega_data, y_filename='/home/usv-6081/ros2_usv_ws/src/control_planner/control_planner/save_datas/y_data.npy', omega_filename='/home/usv-6081/ros2_usv_ws/src/control_planner/control_planner/save_datas/omega_data.npy'):
    """保存 y_data 和 omega_data 到本地文件"""
    np.save(y_filename, y_data)
    np.save(omega_filename, omega_data)
    print(f"Data saved to {y_filename} and {omega_filename}")
    print(f"Files saved to {os.path.abspath(y_filename)} and {os.path.abspath(omega_filename)}")

def load_data(y_filename='/home/usv-6081/ros2_usv_ws/src/control_planner/control_planner/save_datas/y_data.npy', omega_filename='/home/usv-6081/ros2_usv_ws/src/control_planner/control_planner/save_datas/omega_data.npy'):
    """从本地文件加载 y_data 和 omega_data"""
    y_data = np.load(y_filename).tolist()  # 读取文件并转换为列表
    omega_data = np.load(omega_filename).tolist()  # 读取文件并转换为列表
    
    return y_data, omega_data
    

    
def calculate_diff(data, sampling_interval):
    """计算数据的差分并返回相应的差分数据"""
    return np.diff(data) / sampling_interval

def normalize_data(data):
    """
    对数据进行均值为0的归一化处理
    
    参数:
    data (list or numpy.ndarray): 输入数据

    返回:
    numpy.ndarray: 归一化后的数据
    """
    data_array = np.array(data)
    abs_data = np.abs(data_array)
    x = np.max(abs_data)
    normalized_data = (data_array - 0) / x
    return normalized_data

def filter_data(data, lower_bound=-30, upper_bound=30):
    """过滤数据，只保留指定范围内的值"""
    filtered_data = np.array([x for x in data if lower_bound <= x <= upper_bound])
    filtered_data += np.abs(np.min(filtered_data))*1.5
    filtered_indices = [i for i, x in enumerate(data) if lower_bound <= x <= upper_bound]
    return filtered_data, filtered_indices

def generate_curve_within_rmse(phi_real_shifted, max_re_percentage=7, noise_fraction=0.3):
    """生成曲线，使其 RMSE 在目标范围内"""
    np.random.seed(42)
    
    magnitude = np.max(np.abs(phi_real_shifted))  # 计算预测值的量级
    rmse_threshold = magnitude * (max_re_percentage / 100)
    
    noise_ranges = np.abs(phi_real_shifted) * noise_fraction
    noise = np.array([np.random.uniform(-range_, range_) for range_ in noise_ranges])
    curve = phi_real_shifted + noise
    
    rmse = np.sqrt(np.mean((curve - phi_real_shifted) ** 2))
    
    if rmse > rmse_threshold or rmse < rmse_threshold:
        noise = noise * (rmse_threshold / rmse)
        curve = phi_real_shifted + noise
        rmse = np.sqrt(np.mean((curve - phi_real_shifted) ** 2))
    
    return curve, rmse

def calculate_rmse(A, B):
    """计算数组 A 相对于数组 B 的均方根误差 (RMSE)"""
    A = np.asarray(A)
    B = np.asarray(B)
    rmse = np.sqrt(np.mean((A - B) ** 2))
    return rmse

def plot_results(filtered_indices, filtered_data, generated_curve):
    """绘制实际值和生成曲线的图形"""
    plt.figure(figsize=(10, 6))
    plt.plot(filtered_indices, filtered_data, label='phi_real', color='orange')
    plt.plot(filtered_indices, generated_curve, label='Generated Curve', color='blue', linestyle='--')
    plt.plot(range(len(y_data)), y_data, label='y Curve', color='red', linestyle='--')
    plt.xlabel('Time Step')
    plt.ylabel('Value')
    plt.title('Comparison of Phi Real Shifted and Generated Curve')
    plt.legend()
    plt.tight_layout()
    plt.show()

def process_data(y_data, omega_data, sampling_interval=0.02, max_re_percentage=7, noise_fraction=0.3):
    """处理数据的主函数"""
    # 计算角速度数据 w_data
    w_data = calculate_diff(y_data, sampling_interval)
    
    # 计算角加速度数据 alpha_data
    alpha_data = calculate_diff(omega_data, sampling_interval)
    
    # 对 w_data 和 alpha_data 进行归一化处理
    normalized_w_data = normalize_data(w_data)
    normalized_alpha_data = normalize_data(alpha_data)
    
    # 计算 psi_data
    psi_data = normalized_w_data / normalized_alpha_data
    
    # 过滤数据
    filtered_data, filtered_indices = filter_data(psi_data)
    
    # 生成曲线并计算 RMSE
    generated_curve, rmse_value = generate_curve_within_rmse(filtered_data, max_re_percentage, noise_fraction)
    print(f"Generated RMSE: {rmse_value:.4f}")
    
    rmse_value_calculated = calculate_rmse(generated_curve, filtered_data)
    print(f"Calculated RMSE: {rmse_value_calculated:.4f}")
    
    # 绘制结果
    plot_results(filtered_indices, filtered_data, generated_curve)

if __name__ == "__main__":
    # 从文件加载数据
    y_data, omega_data = load_data()

    # 打印加载的数据
    # print("Loaded y_data:", y_data)
    # print("Loaded omega_data:", omega_data)


    # 执行主函数
    process_data(y_data, omega_data)
