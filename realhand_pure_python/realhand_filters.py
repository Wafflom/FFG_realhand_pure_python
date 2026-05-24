import numpy as np
from typing import List, Optional
from collections import deque

class LCFilter:
    """
    LC低通滤波器（一阶低通滤波器）
    离散时间实现，常用于信号平滑
    """
    
    def __init__(self, alpha: float = 0.1, initial_value: float = 0.0):
        """
        初始化LC滤波器
        
        参数：
            alpha: 滤波系数 (0 < alpha <= 1)
                   alpha越小，滤波效果越强（更平滑）
                   alpha越大，响应越快（更灵敏）
            initial_value: 初始值
        """
        if alpha <= 0 or alpha > 1:
            raise ValueError("alpha必须在(0, 1]范围内")
        
        self.alpha = alpha
        self.filtered_value = initial_value
        self.previous_raw = initial_value
        self.previous_filtered = initial_value
        
        # 历史记录（可选，用于调试）
        self.history_raw = []
        self.history_filtered = []
    
    def update(self, new_value: float) -> float:
        """
        更新滤波器并返回滤波后的值
        
        公式：y[n] = α * x[n] + (1-α) * y[n-1]
        其中：x[n]是当前输入，y[n-1]是上一次输出
        
        参数：
            new_value: 新的输入值
        返回：
            滤波后的值
        """
        # 保存历史值
        self.previous_raw = new_value
        self.previous_filtered = self.filtered_value
        
        # LC滤波公式
        self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value
        
        # 记录历史（可选）
        self.history_raw.append(new_value)
        self.history_filtered.append(self.filtered_value)
        
        return self.filtered_value
    
    def update_array(self, new_values: List[float]) -> List[float]:
        """
        批量更新数组
        
        参数：
            new_values: 新的输入值列表
        返回：
            滤波后的值列表
        """
        filtered_values = []
        for value in new_values:
            filtered = self.update(value)
            filtered_values.append(filtered)
        return filtered_values
    
    def reset(self, initial_value: float = 0.0):
        """重置滤波器状态"""
        self.filtered_value = initial_value
        self.previous_raw = initial_value
        self.previous_filtered = initial_value
        self.history_raw = []
        self.history_filtered = []
    
    def get_state(self):
        """获取当前状态"""
        return {
            'filtered_value': self.filtered_value,
            'alpha': self.alpha,
            'history_length': len(self.history_raw)
        }


class MultiChannelLCFilter:
    """
    多通道LC滤波器
    同时对多个信号进行滤波
    """
    
    def __init__(self, num_channels: int, alpha: float = 0.1, 
                 initial_values: Optional[List[float]] = None):
        """
        初始化多通道滤波器
        
        参数：
            num_channels: 通道数量
            alpha: 滤波系数
            initial_values: 初始值列表，长度需等于num_channels
        """
        self.num_channels = num_channels
        self.alpha = alpha
        
        if initial_values is None:
            initial_values = [0.0] * num_channels
        elif len(initial_values) != num_channels:
            raise ValueError(f"初始值长度必须等于通道数 {num_channels}")
        
        # 为每个通道创建一个滤波器
        self.filters = [LCFilter(alpha, initial_values[i]) for i in range(num_channels)]
    
    def update(self, new_values: List[float]) -> List[float]:
        """
        更新所有通道
        
        参数：
            new_values: 新的输入值列表，长度需等于num_channels
        返回：
            滤波后的值列表
        """
        if len(new_values) != self.num_channels:
            raise ValueError(f"输入值长度必须等于通道数 {self.num_channels}")
        
        filtered_values = []
        for i in range(self.num_channels):
            filtered = self.filters[i].update(new_values[i])
            filtered_values.append(filtered)
        
        return filtered_values
    
    def update_channel(self, channel_idx: int, new_value: float) -> float:
        """
        更新单个通道
        
        参数：
            channel_idx: 通道索引 (0-based)
            new_value: 新的输入值
        返回：
            滤波后的值
        """
        if channel_idx < 0 or channel_idx >= self.num_channels:
            raise ValueError(f"通道索引必须在[0, {self.num_channels-1}]范围内")
        
        return self.filters[channel_idx].update(new_value)
    
    def reset(self, initial_values: Optional[List[float]] = None):
        """重置所有通道"""
        if initial_values is None:
            initial_values = [0.0] * self.num_channels
        
        for i in range(self.num_channels):
            self.filters[i].reset(initial_values[i])
    
    def get_state(self):
        """获取所有通道的状态"""
        states = []
        for i, filter_obj in enumerate(self.filters):
            state = filter_obj.get_state()
            state['channel'] = i
            states.append(state)
        return states


class AdaptiveLCFilter(LCFilter):
    """
    自适应LC滤波器
    根据信号变化自动调整alpha值
    """
    
    def __init__(self, alpha_min: float = 0.05, alpha_max: float = 0.3,
                 change_threshold: float = 0.1, initial_value: float = 0.0):
        """
        初始化自适应滤波器
        
        参数：
            alpha_min: 最小alpha值（信号稳定时使用）
            alpha_max: 最大alpha值（信号快速变化时使用）
            change_threshold: 变化阈值，超过此阈值认为信号在快速变化
            initial_value: 初始值
        """
        super().__init__(alpha_max, initial_value)  # 初始使用最大alpha
        self.alpha_min = alpha_min
        self.alpha_max = alpha_max
        self.change_threshold = change_threshold
    
    def update(self, new_value: float) -> float:
        """
        自适应更新滤波器
        
        策略：如果信号变化大，使用较大的alpha快速响应
              如果信号稳定，使用较小的alpha平滑滤波
        """
        # 计算信号变化量
        change_amount = abs(new_value - self.previous_raw)
        
        # 自适应调整alpha
        if change_amount > self.change_threshold:
            # 信号快速变化，使用大alpha快速响应
            self.alpha = self.alpha_max
        else:
            # 信号稳定，使用小alpha平滑滤波
            self.alpha = self.alpha_min
        
        # 调用父类更新方法
        return super().update(new_value)


def apply_lc_filter(data: List[float], alpha: float = 0.1) -> List[float]:
    """
    对数据应用LC滤波（函数式版本）
    
    参数：
        data: 输入数据列表
        alpha: 滤波系数
    返回：
        滤波后的数据列表
    """
    if alpha <= 0 or alpha > 1:
        raise ValueError("alpha必须在(0, 1]范围内")
    
    if not data:
        return []
    
    filtered = [data[0]]  # 第一个值直接使用
    
    for i in range(1, len(data)):
        # LC滤波公式
        y = alpha * data[i] + (1 - alpha) * filtered[i-1]
        filtered.append(y)
    
    return filtered

class KalmanFilter:
    """
    卡尔曼滤波器（简化版）
    用于一维信号的滤波
    """
    
    def __init__(self, 
                 process_variance: float = 1e-5,
                 measurement_variance: float = 0.1,
                 initial_value: float = 0.0,
                 initial_estimate_error: float = 1.0):
        """
        初始化卡尔曼滤波器
        
        参数：
            process_variance: 过程噪声方差（Q，系统不确定性）
            measurement_variance: 测量噪声方差（R，传感器噪声）
            initial_value: 初始状态估计值
            initial_estimate_error: 初始估计误差协方差
        """
        # 系统模型（简单的一维模型）
        self.process_variance = process_variance  # Q
        self.measurement_variance = measurement_variance  # R
        
        # 状态估计
        self.x_hat = initial_value  # 状态估计值
        self.p = initial_estimate_error  # 估计误差协方差
        
        # 历史记录（可选）
        self.history_measurement = []
        self.history_estimate = []
        self.history_kalman_gain = []
    
    def update(self, measurement: float) -> float:
        """
        卡尔曼滤波更新步骤
        
        参数：
            measurement: 测量值
        返回：
            滤波后的估计值
        """
        # 1. 预测步骤
        # 对于简单的一维模型，假设状态不变
        x_hat_minus = self.x_hat  # 先验状态估计
        p_minus = self.p + self.process_variance  # 先验估计误差
        
        # 2. 更新步骤
        # 计算卡尔曼增益
        k = p_minus / (p_minus + self.measurement_variance)  # 卡尔曼增益
        
        # 更新状态估计
        self.x_hat = x_hat_minus + k * (measurement - x_hat_minus)
        
        # 更新估计误差协方差
        self.p = (1 - k) * p_minus
        
        # 记录历史
        self.history_measurement.append(measurement)
        self.history_estimate.append(self.x_hat)
        self.history_kalman_gain.append(k)
        
        return self.x_hat
    
    def update_batch(self, measurements: List[float]) -> List[float]:
        """
        批量更新
        
        参数：
            measurements: 测量值列表
        返回：
            滤波后的估计值列表
        """
        estimates = []
        for measurement in measurements:
            estimate = self.update(measurement)
            estimates.append(estimate)
        return estimates
    
    def reset(self, 
              initial_value: float = 0.0,
              initial_estimate_error: float = 1.0):
        """
        重置滤波器状态
        """
        self.x_hat = initial_value
        self.p = initial_estimate_error
        self.history_measurement = []
        self.history_estimate = []
        self.history_kalman_gain = []
    
    def get_state(self) -> dict:
        """
        获取当前状态
        """
        return {
            'estimate': self.x_hat,
            'error_covariance': self.p,
            'process_variance': self.process_variance,
            'measurement_variance': self.measurement_variance
        }


class MultiChannelKalmanFilter:
    """
    多通道卡尔曼滤波器
    同时对多个独立信号进行滤波
    """
    
    def __init__(self, 
                 num_channels: int,
                 process_variance: float = 1e-5,
                 measurement_variance: float = 0.1,
                 initial_values: Optional[List[float]] = None):
        """
        初始化多通道卡尔曼滤波器
        
        参数：
            num_channels: 通道数量
            process_variance: 过程噪声方差
            measurement_variance: 测量噪声方差
            initial_values: 初始值列表
        """
        self.num_channels = num_channels
        
        if initial_values is None:
            initial_values = [0.0] * num_channels
        elif len(initial_values) != num_channels:
            raise ValueError(f"初始值长度必须等于通道数 {num_channels}")
        
        # 为每个通道创建独立的卡尔曼滤波器
        self.filters = [
            KalmanFilter(
                process_variance=process_variance,
                measurement_variance=measurement_variance,
                initial_value=initial_values[i],
                initial_estimate_error=1.0
            ) for i in range(num_channels)
        ]
    
    def update(self, measurements: List[float]) -> List[float]:
        """
        更新所有通道
        
        参数：
            measurements: 测量值列表，长度需等于num_channels
        返回：
            滤波后的估计值列表
        """
        if len(measurements) != self.num_channels:
            raise ValueError(f"测量值长度必须等于通道数 {self.num_channels}")
        
        estimates = []
        for i in range(self.num_channels):
            estimate = self.filters[i].update(measurements[i])
            estimates.append(estimate)
        
        return estimates
    
    def update_channel(self, channel_idx: int, measurement: float) -> float:
        """
        更新单个通道
        
        参数：
            channel_idx: 通道索引
            measurement: 测量值
        返回：
            滤波后的估计值
        """
        if channel_idx < 0 or channel_idx >= self.num_channels:
            raise ValueError(f"通道索引必须在[0, {self.num_channels-1}]范围内")
        
        return self.filters[channel_idx].update(measurement)
    
    def reset(self, initial_values: Optional[List[float]] = None):
        """
        重置所有通道
        """
        if initial_values is None:
            initial_values = [0.0] * self.num_channels
        
        for i in range(self.num_channels):
            self.filters[i].reset(
                initial_value=initial_values[i],
                initial_estimate_error=1.0
            )
    
    def get_state(self, channel_idx: Optional[int] = None) -> dict:
        """
        获取状态信息
        """
        if channel_idx is not None:
            if channel_idx < 0 or channel_idx >= self.num_channels:
                raise ValueError(f"通道索引必须在[0, {self.num_channels-1}]范围内")
            return self.filters[channel_idx].get_state()
        else:
            states = []
            for i, filter_obj in enumerate(self.filters):
                state = filter_obj.get_state()
                state['channel'] = i
                states.append(state)
            return {'channels': states}


class AdaptiveKalmanFilter(KalmanFilter):
    """
    自适应卡尔曼滤波器
    根据测量噪声自动调整参数
    """
    
    def __init__(self,
                 min_process_variance: float = 1e-6,
                 max_process_variance: float = 1e-3,
                 initial_measurement_variance: float = 0.1,
                 adaptation_rate: float = 0.01,
                 initial_value: float = 0.0):
        """
        初始化自适应卡尔曼滤波器
        
        参数：
            min_process_variance: 最小过程噪声方差
            max_process_variance: 最大过程噪声方差
            initial_measurement_variance: 初始测量噪声方差
            adaptation_rate: 自适应调整速率
        """
        super().__init__(
            process_variance=(min_process_variance + max_process_variance) / 2,
            measurement_variance=initial_measurement_variance,
            initial_value=initial_value
        )
        
        self.min_process_variance = min_process_variance
        self.max_process_variance = max_process_variance
        self.adaptation_rate = adaptation_rate
        self.measurement_history = []
        
    def update(self, measurement: float) -> float:
        """
        自适应更新
        """
        # 保存测量历史
        self.measurement_history.append(measurement)
        if len(self.measurement_history) > 10:
            self.measurement_history.pop(0)
        
        # 计算最近的测量噪声
        if len(self.measurement_history) >= 5:
            recent_std = np.std(self.measurement_history[-5:])
            # 根据噪声水平调整过程噪声方差
            if recent_std > 0.1:
                # 噪声大，增加过程噪声方差
                self.process_variance = min(
                    self.process_variance * (1 + self.adaptation_rate),
                    self.max_process_variance
                )
            else:
                # 噪声小，减小过程噪声方差
                self.process_variance = max(
                    self.process_variance * (1 - self.adaptation_rate),
                    self.min_process_variance
                )
        
        # 调用父类更新方法
        return super().update(measurement)


class SavitzkyGolayFilter:
    """
    Savitzky-Golay滤波器（实时版本）
    适合保留波形特征的平滑
    """
    
    def __init__(self, window_length: int = 7, polyorder: int = 2, 
                 deriv: int = 0, delta: float = 1.0):
        """
        初始化Savitzky-Golay滤波器
        
        参数：
            window_length: 窗口长度（必须为奇数，且大于polyorder）
            polyorder: 多项式阶数
            deriv: 微分阶数（0表示平滑，1表示一阶导等）
            delta: 采样间隔
        """
        if window_length % 2 == 0:
            raise ValueError("window_length必须是奇数")
        if window_length <= polyorder:
            raise ValueError("window_length必须大于polyorder")
        
        self.window_length = window_length
        self.polyorder = polyorder
        self.deriv = deriv
        self.delta = delta
        
        # 数据缓冲区
        self.buffer = deque(maxlen=window_length)
        
        # 计算滤波器系数
        self.coefficients = self._compute_coefficients()
        
        # 历史记录
        self.history_input = []
        self.history_output = []
    
    def _compute_coefficients(self) -> np.ndarray:
        """
        计算Savitzky-Golay滤波器系数
        
        返回：
            滤波器系数数组
        """
        # 简单实现：使用滑动窗口多项式拟合
        # 对于实时应用，我们只需要中心点的系数
        half_window = self.window_length // 2
        
        # 构建范德蒙矩阵
        x = np.arange(-half_window, half_window + 1, dtype=float)
        A = np.vander(x, self.polyorder + 1, increasing=True)
        
        # 使用最小二乘法求解系数
        # 对于Savitzky-Golay，我们只需要中心点的拟合值
        # 这相当于取A的伪逆的第一行
        coeff = np.linalg.pinv(A)[self.deriv]
        
        # 考虑微分和采样间隔
        if self.deriv > 0:
            for i in range(self.deriv):
                coeff = np.polyder(coeff)
            coeff = coeff / (self.delta ** self.deriv)
        
        return coeff
    
    def update(self, new_value: float) -> float:
        """
        更新滤波器并返回滤波后的值
        
        参数：
            new_value: 新的输入值
        返回：
            滤波后的值
        """
        # 添加到缓冲区
        self.buffer.append(new_value)
        
        # 如果缓冲区未满，直接返回原值
        if len(self.buffer) < self.window_length:
            self.history_input.append(new_value)
            self.history_output.append(new_value)
            return new_value
        
        # 应用Savitzky-Golay滤波
        # 将缓冲区转换为数组
        window_data = np.array(self.buffer)
        
        # 使用预计算的系数进行卷积
        filtered_value = np.dot(window_data, self.coefficients)
        
        # 记录历史
        self.history_input.append(new_value)
        self.history_output.append(filtered_value)
        
        # 限制历史长度
        max_history = 1000
        if len(self.history_input) > max_history:
            self.history_input = self.history_input[-max_history:]
            self.history_output = self.history_output[-max_history:]
        
        return filtered_value
    
    def update_batch(self, new_values: List[float]) -> List[float]:
        """
        批量更新
        
        参数：
            new_values: 新的输入值列表
        返回：
            滤波后的值列表
        """
        filtered_values = []
        for value in new_values:
            filtered = self.update(value)
            filtered_values.append(filtered)
        return filtered_values
    
    def reset(self):
        """重置滤波器状态"""
        self.buffer.clear()
        self.history_input = []
        self.history_output = []
    
    def get_state(self) -> dict:
        """获取当前状态"""
        return {
            'window_length': self.window_length,
            'polyorder': self.polyorder,
            'deriv': self.deriv,
            'buffer_size': len(self.buffer),
            'coefficients': self.coefficients.tolist()
        }


class MultiChannelSavitzkyGolayFilter:
    """
    多通道Savitzky-Golay滤波器
    """
    
    def __init__(self, num_channels: int, 
                 window_length: int = 7, polyorder: int = 2,
                 initial_values: Optional[List[float]] = None):
        """
        初始化多通道滤波器
        
        参数：
            num_channels: 通道数量
            window_length: 窗口长度
            polyorder: 多项式阶数
            initial_values: 初始值列表
        """
        self.num_channels = num_channels
        
        if initial_values is None:
            initial_values = [0.0] * num_channels
        elif len(initial_values) != num_channels:
            raise ValueError(f"初始值长度必须等于通道数 {num_channels}")
        
        # 为每个通道创建滤波器
        self.filters = []
        for i in range(num_channels):
            filter_obj = SavitzkyGolayFilter(
                window_length=window_length,
                polyorder=polyorder
            )
            # 用初始值填充缓冲区
            for _ in range(window_length // 2):
                filter_obj.update(initial_values[i])
            self.filters.append(filter_obj)
    
    def update(self, new_values: List[float]) -> List[float]:
        """
        更新所有通道
        
        参数：
            new_values: 新的输入值列表
        返回：
            滤波后的值列表
        """
        if len(new_values) != self.num_channels:
            raise ValueError(f"输入值长度必须等于通道数 {self.num_channels}")
        
        filtered_values = []
        for i in range(self.num_channels):
            filtered = self.filters[i].update(new_values[i])
            filtered_values.append(filtered)
        
        return filtered_values
    
    def reset(self, initial_values: Optional[List[float]] = None):
        """重置所有通道"""
        if initial_values is None:
            initial_values = [0.0] * self.num_channels
        
        for i in range(self.num_channels):
            self.filters[i].reset()
            # 用初始值预热
            for _ in range(self.filters[i].window_length // 2):
                self.filters[i].update(initial_values[i])


class AdaptiveSavitzkyGolayFilter:
    """
    自适应Savitzky-Golay滤波器
    根据信号特性自动调整参数
    """
    
    def __init__(self, 
                 min_window: int = 5,
                 max_window: int = 15,
                 base_polyorder: int = 2,
                 noise_threshold: float = 0.05,
                 initial_value: float = 0.0):
        """
        初始化自适应滤波器
        
        参数：
            min_window: 最小窗口长度
            max_window: 最大窗口长度
            base_polyorder: 基础多项式阶数
            noise_threshold: 噪声阈值
            initial_value: 初始值
        """
        self.min_window = min_window
        self.max_window = max_window
        self.base_polyorder = base_polyorder
        self.noise_threshold = noise_threshold
        
        # 当前滤波器
        self.current_filter = SavitzkyGolayFilter(
            window_length=(min_window + max_window) // 2,
            polyorder=base_polyorder
        )
        
        # 信号特性跟踪
        self.signal_buffer = deque(maxlen=20)
        self.current_noise_level = 0.0
        
    def update(self, new_value: float) -> float:
        """
        自适应更新
        """
        # 更新信号缓冲区
        self.signal_buffer.append(new_value)
        
        # 计算信号特性（噪声水平）
        if len(self.signal_buffer) >= 10:
            recent_data = np.array(self.signal_buffer)
            self.current_noise_level = np.std(recent_data)
        
        # 根据噪声水平调整窗口大小
        if len(self.signal_buffer) >= 5:
            if self.current_noise_level > self.noise_threshold * 2:
                # 高噪声，使用大窗口强滤波
                new_window = self.max_window
            elif self.current_noise_level > self.noise_threshold:
                # 中等噪声，使用中等窗口
                new_window = (self.min_window + self.max_window) // 2
            else:
                # 低噪声，使用小窗口保留细节
                new_window = self.min_window
            
            # 如果窗口大小需要改变，创建新滤波器
            if new_window != self.current_filter.window_length:
                # 获取当前滤波器的输出作为新滤波器的初始状态
                current_output = self.current_filter.update(new_value)
                
                # 创建新滤波器
                self.current_filter = SavitzkyGolayFilter(
                    window_length=new_window,
                    polyorder=min(self.base_polyorder, new_window - 1)
                )
                
                # 用当前输出预热新滤波器
                for _ in range(new_window // 2):
                    self.current_filter.update(current_output)
                
                return current_output
        
        # 使用当前滤波器
        return self.current_filter.update(new_value)