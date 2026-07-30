"""
多态线性映射器
支持任意数量的状态
"""
import numpy as np
try:
    from colorama import Fore, init
except ImportError:
    class _PlainColor:
        RED = ""
        GREEN = ""
        YELLOW = ""
        BLUE = ""
        CYAN = ""
        RESET = ""
        RESET_ALL = ""

    Fore = _PlainColor()

    def init(*args, **kwargs):
        return None
from typing import List, Dict
from .realhand_filters import MultiChannelKalmanFilter

class MultiStateLinearMapper:
    """
    多态线性映射器
    支持任意数量的手势状态
    """
    
    def __init__(self,FINGER_CONFIGS,MAPPING_ORDER,is_debug = False):
        self.finger_configs = FINGER_CONFIGS.copy()
        self.mapping_order = MAPPING_ORDER.copy()
        
        # 状态存储
        self.glove_states = {}  # {状态名: 手套角度数组}
        self.robot_states = {}  # {状态名: 机械手角度数组}
        self.state_order = []   # 状态顺序列表
        self.debug_value = [0.0] * 20  # 长度20的debug缓冲数据
        self.isdebug = is_debug
        self.debug_fingers = None  # None=全部, []=全部, ["finger_name"]=指定手指

        # 输入侧卡尔曼滤波（realhand_filters 还提供 LC / Savitzky-Golay 备选）
        num_joints = 21
        self.filters = MultiChannelKalmanFilter(
            num_channels=num_joints,
            process_variance=1e-5,
            measurement_variance=0.0005,
            initial_values=[0.0] * num_joints
        )

        # 历史记录（用于调试和可视化）
        self.raw_history = []
        self.filtered_history = []
    
    def add_state(self, state_name: str,
                  glove_angles: List[float],
                  robot_angles: List[float]):
        """
        添加一个手势状态
        
        参数：
            state_name: 状态名称，如 'original', 'opose', 'fist'等
            glove_angles: 手套角度 (21维)
            robot_angles: 机械手角度 (11维)
        """
        self.glove_states[state_name] = np.array(glove_angles)
        self.robot_states[state_name] = np.array(robot_angles)
        
        if state_name not in self.state_order:
            self.state_order.append(state_name)
    
    def remove_state(self, state_name: str):
        """移除一个状态"""
        if state_name in self.glove_states:
            del self.glove_states[state_name]
            del self.robot_states[state_name]
            if state_name in self.state_order:
                self.state_order.remove(state_name)
    
    def set_state_order(self, state_order: List[str]):
        """
        设置状态顺序（从原始到最弯曲）
        
        示例：
            ['original', 'opose', 'fist']
        """
        # 验证所有状态都存在
        for state in state_order:
            if state not in self.glove_states:
                raise ValueError(f"状态 '{state}' 未定义")
        
        self.state_order = state_order
    
    def map_glove_to_robot(self, glove_current):
        """
        动态权重映射
        在映射过程中根据其他手指状态调整权重
        """
        
        if isinstance(glove_current, np.ndarray):
            glove_current = glove_current.tolist()
        elif isinstance(glove_current, list):
            glove_current = glove_current
        else:
            glove_current = list(glove_current)

        if len(self.state_order) < 2:
            raise ValueError("请至少设置两个状态")
        
        if 'original' not in self.glove_states:
            raise ValueError("必须包含 'original' 状态作为基准")
        
        glove_current_arr = np.array(glove_current)
        robot_angles = self.robot_states['original'].copy()
        
        for config_name in self.mapping_order:
            config = self.finger_configs[config_name]
            angle = self._map_finger_multi_state(glove_current_arr, config)
            robot_angles[config['robot_idx']] = angle

        # self.debug_value[config['robot_idx']] = angle

        
        
        return robot_angles
    
    def _map_finger_multi_state(self, glove_current: np.ndarray, 
                               config: dict) -> float:
        """
        多状态手指映射
        """
        joints = config['joints']
        weights = self._normalize_weights(config['weights'])
        robot_idx = config['robot_idx']
        
        # 计算当前融合值
        current_fused = self._calculate_fused_value(
            glove_current, joints, weights
        )
        # self.debug_value[robot_idx] = current_fused
        
        # 计算所有状态的融合值
        state_fused_values = {}
        for state_name in self.state_order:
            fused = self._calculate_reference_fused(
                joints, weights, self.glove_states[state_name]
            )
            state_fused_values[state_name] = fused
        
        # 获取所有状态的角度
        state_angles = {}
        for state_name in self.state_order:
            state_angles[state_name] = self.robot_states[state_name][robot_idx]
        
        # 分段线性插值
        result_angle = self._multi_state_interpolation(
            current_fused, state_fused_values, state_angles
        )
        
        # 处理反向运动
        if config.get('reverse_motion', True):
            # 找到最小和最大角度
            min_angle = min(state_angles.values())
            max_angle = max(state_angles.values())
            
            result_angle = max_angle - (result_angle - min_angle)
            # print("触发反向运动")
        
        return result_angle
    
    def _calculate_fused_value(self, data: np.ndarray,
                            joints: List[int],
                            weights) -> float:
        """
        完整的融合值计算，处理上下限越界
        """
        # 确保有原始状态
        if 'original' not in self.glove_states:
            return 0.0
        
        original = self.glove_states['original']
        weights = np.array(weights)
        
        # 归一化权重
        if np.sum(weights) > 0:
            weights = weights / np.sum(weights)
        
        fused = 0.0
        
        for i, idx in enumerate(joints):
            # 获取当前值和原始值
            current = data[idx]
            orig = original[idx]
            
            # 步骤1: 找到该关节在所有状态中的最小值和最大值
            all_vals = [orig]
            for state_data in self.glove_states.values():
                all_vals.append(state_data[idx])
            
            min_val = min(all_vals)
            max_val = max(all_vals)
            
            # 步骤2: 截断当前值到[min_val, max_val]范围
            clamped = np.clip(current, min_val, max_val)
            
            # 步骤3: 计算归一化位置
            if abs(max_val - min_val) < 1e-6:
                normalized_diff = 0.0
            else:
                orig_norm = (orig - min_val) / (max_val - min_val)
                clamped_norm = (clamped - min_val) / (max_val - min_val)
                normalized_diff = abs(clamped_norm - orig_norm)
            
            fused += weights[i] * normalized_diff
        return fused
    
    def _calculate_reference_fused(self, joints: List[int],
                                  weights: np.ndarray,
                                  reference_data: np.ndarray) -> float:
        """
        计算参考融合值
        """
        return self._calculate_fused_value(reference_data, joints, weights)
    
    def _multi_state_interpolation(self, current_fused: float,
                                  state_fused_values: Dict[str, float],
                                  state_angles: Dict[str, float]) -> float:
        """
        多状态分段线性插值
        """
        # 确保状态顺序正确
        if not self.state_order:
            return 0.0
        
        # 处理边界情况
        if current_fused <= state_fused_values[self.state_order[0]]:
            return state_angles[self.state_order[0]]
        
        if current_fused >= state_fused_values[self.state_order[-1]]:
            return state_angles[self.state_order[-1]]
        
        # 找到当前融合值所在区间
        for i in range(len(self.state_order) - 1):
            state1 = self.state_order[i]
            state2 = self.state_order[i + 1]
            
            fused1 = state_fused_values[state1]
            fused2 = state_fused_values[state2]
            
            # 确保区间有效
            if fused1 <= current_fused <= fused2:
                if fused2 - fused1 > 1e-6:
                    t = (current_fused - fused1) / (fused2 - fused1)
                else:
                    t = 0.0
                
                angle1 = state_angles[state1]
                angle2 = state_angles[state2]
                return angle1 + t * (angle2 - angle1)
        
        # 如果没有找到区间（理论上不会发生），返回最近状态的角度
        min_diff = float('inf')
        nearest_angle = 0.0
        for state_name in self.state_order:
            diff = abs(current_fused - state_fused_values[state_name])
            if diff < min_diff:
                min_diff = diff
                nearest_angle = state_angles[state_name]
        
        return nearest_angle
    
    def _normalize_weights(self, weights: List[float]) -> List[float]:
        """
        归一化权重
        """
        if hasattr(weights, 'tolist'):
            # 如果是 NumPy 数组
            weight_list = weights.tolist()
        elif isinstance(weights, list):
            # 如果已经是列表
            weight_list = weights
        else:
            # 其他情况，尝试转换
            weight_list = list(weights)
        total = np.sum(weight_list)
        if total > 0:
            result_array = weight_list / total
        else:
            result_array = weight_list
        
        # 关键：转换回列表
        return result_array.tolist()
    
    def get_state_info(self) -> Dict:
        """
        获取状态信息
        """
        # 基础信息
        info = {
            'states': list(self.glove_states.keys()),
            'state_order': self.state_order,
            'has_original': 'original' in self.glove_states
        }
        
        return info
       
    
    def clear_states(self):
        """清除所有状态"""
        self.glove_states.clear()
        self.robot_states.clear()
        self.state_order.clear()

    def set_debug(self, enabled):
        """
        设置 debug 模式
        
        Args:
            enabled: bool 或 list
                - True: 开启调试，显示全部手指
                - False: 关闭调试
                - []: 开启调试，显示全部手指
                - ["finger_name", ...]: 开启调试，只显示指定手指
        """
        if isinstance(enabled, bool):
            self.isdebug = enabled
            self.debug_fingers = None
        elif isinstance(enabled, list):
            self.isdebug = True
            self.debug_fingers = enabled if enabled else None
        else:
            self.isdebug = bool(enabled)
            self.debug_fingers = None

    def _should_debug(self, finger_name: str) -> bool:
        """检查是否应该输出该手指的调试信息"""
        if not self.isdebug:
            return False
        if self.debug_fingers is None:
            return True
        return finger_name in self.debug_fingers


class DynamicWeightMultiStateLinearMapper(MultiStateLinearMapper):
    """
    动态权重多态线性映射器
    继承自MultiStateLinearMapper，增加动态权重调整功能
    增加扩展线性映射功能：基于open/opose线性映射，可以继续延伸
    """
    
    def __init__(self, FINGER_CONFIGS, MAPPING_ORDER,is_debug=False):
        super().__init__(FINGER_CONFIGS, MAPPING_ORDER,is_debug)
        
        # 动态权重配置
        self.dynamic_weight_configs = {}
        
        # 扩展映射配置
        self.extended_mapping_enabled = {}
        self.scale_factors = {}
        self.exp_factors = {}
        # self.isdebug = is_debug
        # 缓存计算过的关节映射值
        self.cached_mapped_values = {}
        
        # 从配置表初始化扩展映射
        self._init_extended_mapping_from_config()

    def _init_extended_mapping_from_config(self):
        """从配置表初始化扩展映射设置"""
        for finger_name, config in self.finger_configs.items():
            if config.get('dynamic_weight'):
                self.set_dynamic_weight_config(finger_name, config['dynamic_weight'])
            ext_config = config.get('extended_mapping')
            if ext_config and ext_config.get('enabled', False):
                self.extended_mapping_enabled[finger_name] = True
                
                # 设置缩放因子
                scale_factor = ext_config.get('scale_factor', 1.0)
                if scale_factor != 1.0:
                    self.scale_factors[finger_name] = scale_factor
                exp_factor = ext_config.get('extended_exp_factor', 1.0)
                if exp_factor != 1.0:
                    self.exp_factors[finger_name] = exp_factor
    
    def set_dynamic_weight_config(self, finger_name: str, config: Dict):
        """
        设置动态权重配置
        """
        self.dynamic_weight_configs[finger_name] = config
    
    def set_extended_mapping(self, finger_name: str, enabled: bool = True, 
                            scale_factor: float = 1.0):
        """
        手动设置扩展映射
        
        参数：
            finger_name: 手指名称
            enabled: 是否启用扩展映射
            scale_factor: 缩放因子，>1加快映射，<1减慢映射
        """
        self.extended_mapping_enabled[finger_name] = enabled
        if scale_factor != 1.0:
            self.scale_factors[finger_name] = scale_factor
    
    def fit_exp_factor(self, finger_name: str, current_fused: float, 
                       fused_open: float, fused_opose: float,
                       angle_open: float, angle_opose: float, angle_fist: float) -> float:
        """
        根据当前握拳值自动拟合延伸因子
        
        目标：使 current_fused 映射到 angle_fist
        
        公式：extension = slope * t * (1 + (exp-1) * t)
        其中 slope = angle_opose - angle_open, t = normalized - 1
        
        参数：
            finger_name: 手指名称
            current_fused: 当前握拳时的融合值
            fused_open: 张开时的融合值
            fused_opose: O型时的融合值
            angle_open: 张开时的机械手角度
            angle_opose: O型时的机械手角度
            angle_fist: 握拳极限时的机械手角度
            
        返回：
            计算出的延伸因子
        """
        if abs(fused_opose - fused_open) < 1e-6:
            return 1.0
        
        normalized = (current_fused - fused_open) / (fused_opose - fused_open)
        
        if normalized <= 1.0:
            return 1.0
        
        t = normalized - 1.0
        
        slope = angle_opose - angle_open
        target_extension = angle_fist - angle_opose
        
        if abs(slope * t) < 1e-6 or abs(target_extension) < 1e-6:
            return 1.0
        
        base_extension = slope * t
        ratio = target_extension / base_extension
        
        exp_factor = (ratio - 1.0) / t + 1.0
        
        return max(1.0, min(100.0, exp_factor))
    
    def _apply_scale_factor(self, fused_value: float, 
                        fused_open: float, fused_opose: float, 
                        finger_name: str) -> float:
        """
        应用缩放因子，基于归一化的[0,1]范围
        
        参数：
            fused_value: 原始融合值
            fused_open: open状态的融合值（映射到0）
            fused_opose: opose状态的融合值（映射到1）
            finger_name: 手指名称
        """
        scale_factor = self.scale_factors.get(finger_name, 1.0)
        
        if scale_factor == 1.0:
            return fused_value
        
        # 将原始融合值归一化到[0,1]范围
        # 融合值范围 [fused_open, fused_opose] -> [0, 1]
        if abs(fused_opose - fused_open) < 1e-6:
            normalized = 0.0
        else:
            normalized = (fused_value - fused_open) / (fused_opose - fused_open)
        
        # 如果已经到达 opose 位置，不应用缩放
        if abs(normalized - 1.0) < 1e-6:
            return fused_value
        
        # 应用缩放因子到归一化的值
        scaled_normalized = normalized * scale_factor
        
        # 将缩放后的归一化值转换回原始融合值范围
        scaled_fused = fused_open + scaled_normalized * (fused_opose - fused_open)
        
        return scaled_fused
    
    def _get_max_angle(self, robot_idx: int) -> float:
        """
        获取关节的最大角度
        如果有fist状态，使用fist状态的角度作为最大角度
        否则使用默认的最大角度
        """
        # 如果有fist状态，使用fist状态的角度
        if 'fist' in self.robot_states:
            return self.robot_states['fist'][robot_idx]
        
        # 默认最大角度（可以根据需要调整）
        return 1.57  # 默认90度
    
    def map_glove_to_robot(self, source_current):
        """
        动态权重映射
        在映射过程中根据其他手指状态调整权重
        """
        self.debug_value[3] = source_current[1]

        glove_current = self.filters.update(source_current)
        # 应用Savitzky-Golay滤波
        # filtered_angles = self.filters.update(robot_angles)
        
        # 记录历史（用于调试和分析）
        self.raw_history.append(source_current.copy())
        self.filtered_history.append(glove_current.copy())

        # filtered_angles = self.filters.update(robot_angles)
        # 限制历史记录长度
        max_history = 100
        if len(self.raw_history) > max_history:
            self.raw_history = self.raw_history[-max_history:]
            self.filtered_history = self.filtered_history[-max_history:]

        self.debug_value[4] = glove_current[1]

        if isinstance(glove_current, np.ndarray):
            glove_current = glove_current.tolist()
        elif isinstance(glove_current, list):
            glove_current = glove_current
        else:
            glove_current = list(glove_current)

        if len(self.state_order) < 2:
            raise ValueError("请至少设置两个状态")
        
        if 'original' not in self.glove_states:
            raise ValueError("必须包含 'original' 状态作为基准")
        
        # 重置缓存
        self.cached_mapped_values = {}
         
        glove_current_arr = np.array(glove_current)
        robot_angles = self.robot_states['original'].copy()
        
        # 第一遍：计算所有需要用于触发判断的手指映射值
        for config_name in self.mapping_order:
            if config_name in self.dynamic_weight_configs:
                trigger_finger = self.dynamic_weight_configs[config_name]['trigger_finger']
                # 先计算触发手指的映射值
                if trigger_finger not in self.cached_mapped_values:
                    trigger_value = self._calculate_trigger_value(
                        glove_current_arr, trigger_finger
                    )
                    self.cached_mapped_values[trigger_finger] = trigger_value
                    
        
        # 第二遍：使用动态权重进行映射
        for config_name in self.mapping_order:
            # 获取动态配置（如果有）
            dynamic_config = self.dynamic_weight_configs.get(config_name)
            
            if dynamic_config:
                # 使用动态权重进行映射
                config = self.finger_configs[config_name]
                angle = self._map_finger_dynamic_weight(
                    glove_current_arr, config_name, dynamic_config, config
                )
            else:
                # 使用多状态方法映射（支持扩展映射）
                config = self.finger_configs[config_name]
                angle = self._map_finger_multi_state(glove_current_arr, config)
            
            robot_idx = self.finger_configs[config_name]['robot_idx']
            robot_angles[robot_idx] = angle


        return robot_angles
    
    def _calculate_trigger_value(self, glove_current: np.ndarray, 
                               trigger_finger: str) -> float:
        """
        计算触发手指的归一化映射值（0-1范围）
        
        返回：
            归一化的映射值，0表示原始状态，1表示最弯曲状态
        """
        if trigger_finger not in self.finger_configs:
            raise ValueError(f"触发手指配置 '{trigger_finger}' 不存在")
        
        config = self.finger_configs[trigger_finger]
        
        # 计算当前融合值
        joints = config['joints']
        weights = self._normalize_weights(config['weights'])

        current_fused = self._calculate_fused_value(
            glove_current, joints, weights
        )
        
        # 计算所有状态的融合值
        state_fused_values = {}
        for state_name in self.state_order:
            fused = self._calculate_reference_fused(
                joints, weights, self.glove_states[state_name]
            )
            state_fused_values[state_name] = fused
        
        # 归一化到0-1范围
        min_fused = min(state_fused_values.values())
        max_fused = max(state_fused_values.values())
        
        if abs(max_fused - min_fused) < 1e-6:
            return 0.0
        
        normalized = (current_fused - min_fused) / (max_fused - min_fused)
        return np.clip(normalized, 0.0, 1.0)
    
    def _map_finger_dynamic_weight(self, glove_current: np.ndarray,
                                  finger_name: str,
                                  dynamic_config: Dict,
                                  base_config: Dict) -> float:
        """
        使用动态权重进行手指映射
        """
        # 获取触发值
        trigger_finger = dynamic_config['trigger_finger']
        if trigger_finger not in self.cached_mapped_values:
            trigger_value = self._calculate_trigger_value(
                glove_current, trigger_finger
            )
            self.cached_mapped_values[trigger_finger] = trigger_value
        else:
            trigger_value = self.cached_mapped_values[trigger_finger]
        
        # 根据阈值选择配置
        threshold = dynamic_config['threshold']
        temp_config = self.finger_configs[finger_name].copy()  # 默认使用基础配置

        if trigger_value < threshold:
            weight_config = dynamic_config['low_weight_config']
            # 创建临时配置
            
            temp_config['joints'] = weight_config['joints']
            temp_config['weights'] = weight_config['weights']
            if 'reverse_motion' in weight_config:
                temp_config['reverse_motion'] = weight_config['reverse_motion']
            else:
                temp_config['reverse_motion'] = base_config.get('reverse_motion', False)      
        else:
          # 使用高权重配置
            weight_config = dynamic_config.get('high_weight_config', {})
            # 创建临时配置，合并基础配置和高权重配置
            if weight_config:  # 如果有高权重配置
                temp_config['joints'] = weight_config.get('joints', temp_config['joints'])
                temp_config['weights'] = weight_config.get('weights', temp_config['weights'])
                # 优先使用高权重配置的reverse_motion
                if 'reverse_motion' in weight_config:
                    temp_config['reverse_motion'] = weight_config['reverse_motion']
        
        # 使用临时配置进行映射（支持扩展映射）
        return self._map_finger_multi_state(glove_current, temp_config)
    
    def _map_finger_multi_state(self, glove_current: np.ndarray,
                               config: dict) -> float:
        """
        手指映射主方法
        支持扩展映射和原始多状态映射
        """
        # 查找手指名称
        finger_name = None
        for name, cfg in self.finger_configs.items():
            if cfg['robot_idx'] == config['robot_idx']:
                finger_name = name
                break
        # print(self.extended_mapping_enabled)
        # 检查是否启用扩展映射
        if (finger_name and finger_name in self.extended_mapping_enabled and 
            self.extended_mapping_enabled[finger_name]):
            # print("触发线性映射")
            return self._map_finger_extended(glove_current, config, finger_name)
        else:
            # 使用原始的多状态映射
            return self._map_finger_original(glove_current, config, finger_name)
    
    def _map_finger_original(self, glove_current: np.ndarray, 
                            config: dict, finger_name: str = None) -> float:
        """
        原始的多状态手指映射
        """
        joints = config['joints']
        weights = self._normalize_weights(config['weights'])
        robot_idx = config['robot_idx']
        
        # 计算当前融合值
        current_fused = self._calculate_fused_value(
            glove_current, joints, weights
        )
        
        # 计算所有状态的融合值
        state_fused_values = {}
        for state_name in self.state_order:
            fused = self._calculate_reference_fused(
                joints, weights, self.glove_states[state_name]
            )
            state_fused_values[state_name] = fused
        
        # 获取所有状态的角度
        state_angles = {}
        for state_name in self.state_order:
            state_angles[state_name] = self.robot_states[state_name][robot_idx]
        
        if self._should_debug(finger_name):
            print(f"\n=== {finger_name} 调试信息 (original) ===")
            print(f"启用状态: {self.state_order}")
            print(f"权重: {config['weights']}")
            joints = config['joints']
            glove_joints_vals = {f"glove[{j}]": glove_current[j] for j in joints}
            print(f"手套数据: {glove_joints_vals}")
            print(f"融合值: {current_fused:.6f}")
            print(f"状态融合值: {state_fused_values}")
            print(f"状态角度: {state_angles}")
        
        # 分段线性插值
        result_angle = self._multi_state_interpolation(
            current_fused, state_fused_values, state_angles
        )
        
        if self._should_debug(finger_name):
            print(f"插值结果: {result_angle:.6f}")
        
        # 处理反向运动
        if config.get('reverse_motion', True):
            # 找到最小和最大角度
            min_angle = min(state_angles.values())
            max_angle = max(state_angles.values())
            
            result_angle = max_angle - (result_angle - min_angle)
            if self._should_debug(finger_name):
                print(f"reverse_motion=True, 反转后: {result_angle:.6f}")
        
        return result_angle
    
    def _map_finger_extended(self, glove_current: np.ndarray,
                            config: dict, finger_name: str) -> float:
        """
        多段映射实现
        
        根据 state_order 决定映射段数：
        - ['origin', 'opose', 'fist'] → 三段映射，截断到 fist
        - ['origin', 'opose'] + extended_mapping.enabled=True → 两段映射，延伸到 fist 截断
        - ['origin', 'opose'] + extended_mapping.enabled=False → 两段映射，截断到 opose
        """
        joints = config['joints']
        weights = self._normalize_weights(config['weights'])
        robot_idx = config['robot_idx']
        
        # 获取启用的状态列表
        states = self.state_order
        num_states = len(states)
        
        if num_states < 2:
            print("警告：状态数量不足，回退到原始映射")
            return self._map_finger_original(glove_current, config)
        
        # 计算当前融合值
        current_fused_raw = self._calculate_fused_value(glove_current, joints, weights)
        
        # 计算第一个和最后一个状态的融合值
        fused_first = self._calculate_reference_fused(joints, weights, self.glove_states[states[0]])
        fused_last = self._calculate_reference_fused(joints, weights, self.glove_states[states[-1]])
        
        # 应用缩放因子
        current_fused = self._apply_scale_factor(current_fused_raw, fused_first, fused_last, finger_name)
        
        # 获取第一个和最后一个状态的角度
        angle_first = self.robot_states[states[0]][robot_idx]
        angle_last = self.robot_states[states[-1]][robot_idx]
        
        # 确保顺序正确
        if angle_first > angle_last:
            angle_first, angle_last = angle_last, angle_first
        
        if self._should_debug(finger_name):
            print(f"\n=== {finger_name} 调试信息 ===")
            print(f"启用状态: {states}")
            print(f"权重: {config['weights']}")
            print(f"原始融合值: {current_fused_raw:.6f}")
            print(f"缩放后融合值: {current_fused:.6f}")
            print(f"融合值范围: [{fused_first:.6f}, {fused_last:.6f}]")
            print(f"机械手角度范围: [{angle_first:.6f}, {angle_last:.6f}]")
        
        # 归一化融合值
        if abs(fused_last - fused_first) < 1e-6:
            normalized_fused = 0.5
        else:
            normalized_fused = (current_fused - fused_first) / (fused_last - fused_first)
        
        if self._should_debug(finger_name):
            print(f"归一化融合值: {normalized_fused:.6f}")
        
        # 判断是否需要延伸（只有 original + opose 两段模式才启用）
        extrapolation_enabled = self.extended_mapping_enabled.get(finger_name, False)
        use_extrapolation = extrapolation_enabled and states == ['original', 'opose']
        
        if num_states >= 3:
            result_angle = self._multi_state_map(joints, weights, robot_idx, current_fused_raw, finger_name)
        elif use_extrapolation:
            result_angle = self._extrapolate_to_fist(
                current_fused, fused_first, fused_last, 
                angle_first, angle_last, robot_idx, finger_name, joints, weights
            )
        else:
            result_angle = self._two_state_map(
                current_fused, fused_first, fused_last,
                angle_first, angle_last, finger_name
            )
        
        if config.get('reverse_motion', False):
            min_angle = min(angle_first, angle_last)
            max_angle = max(angle_first, angle_last)
            clamped = np.clip(result_angle, min_angle, max_angle)
            result_angle = max_angle - (clamped - min_angle)
        
        return result_angle
    
    def _multi_state_map(self, joints, weights, robot_idx, current_fused, finger_name):
        """多段映射：使用所有启用的状态进行分段插值"""
        state_fused_values = {}
        state_angles = {}
        
        for state_name in self.state_order:
            fused = self._calculate_reference_fused(joints, weights, self.glove_states[state_name])
            state_fused_values[state_name] = fused
            state_angles[state_name] = self.robot_states[state_name][robot_idx]
        
        result_angle = self._multi_state_interpolation(current_fused, state_fused_values, state_angles)
        
        if self._should_debug(finger_name):
            print(f"多段映射结果: {result_angle:.6f}")
        
        return result_angle
    
    def _extrapolate_to_fist(self, current_fused, fused_first, fused_last, 
                             angle_first, angle_last, robot_idx, finger_name, joints, weights):
        """两段映射 + 延伸映射，截断到 fist 角度"""
        if abs(fused_last - fused_first) < 1e-6:
            normalized = 0.5
        else:
            normalized = (current_fused - fused_first) / (fused_last - fused_first)
        
        if self._should_debug(finger_name):
            print(f"归一化融合值: {normalized:.6f}")
        
        exp_factor = self.exp_factors.get(finger_name, 1.0)
        slope = angle_last - angle_first
        
        if normalized <= 0:
            result_angle = angle_first
            if self._should_debug(finger_name):
                print(f"归一化值<=0: result_angle={result_angle:.6f}")
        elif normalized <= 1:
            result_angle = angle_first + normalized * slope
            if self._should_debug(finger_name):
                print(f"归一化值在[0,1]: result_angle={result_angle:.6f}")
        else:
            t = normalized - 1.0
            extension = slope * t * (1.0 + (exp_factor - 1.0) * t)
            result_angle = angle_last + extension
            
            if self._should_debug(finger_name):
                print(f"延伸: normalized={normalized:.6f}, t={t:.4f}, exp_factor={exp_factor:.2f}, result={result_angle:.6f}")
        
        if 'fist' in self.robot_states:
            angle_fist = self.robot_states['fist'][robot_idx]
            if slope > 0:
                result_angle = min(result_angle, angle_fist)
            else:
                result_angle = max(result_angle, angle_fist)
            if self._should_debug(finger_name):
                print(f"截断到fist: angle_fist={angle_fist:.6f}, result={result_angle:.6f}")
        
        return result_angle
    
    def _two_state_map(self, current_fused, fused_first, fused_last,
                       angle_first, angle_last, finger_name):
        """两段映射：线性插值并截断到最后一个状态"""
        if abs(fused_last - fused_first) < 1e-6:
            normalized = 0.5
        else:
            normalized = (current_fused - fused_first) / (fused_last - fused_first)
        
        # 截断到 [0, 1]
        normalized = max(0.0, min(1.0, normalized))
        
        result_angle = angle_first + normalized * (angle_last - angle_first)
        
        if self._should_debug(finger_name):
            print(f"两段映射截断: normalized={normalized:.6f}, result={result_angle:.6f}")
        
        return result_angle
        
        return result_angle
    
    def get_mapping_info(self, finger_name: str = None) -> Dict:
        """
        获取映射信息
        """
        if finger_name:
            return self._get_finger_info(finger_name)
        else:
            return {name: self._get_finger_info(name) for name in self.finger_configs}
    
    def _get_finger_info(self, finger_name: str) -> Dict:
        """获取单个手指的信息"""
        if finger_name not in self.finger_configs:
            return {}
        
        robot_idx = self.finger_configs[finger_name]['robot_idx']
        max_angle = self._get_max_angle(robot_idx)
        
        info = {
            'name': self.finger_configs[finger_name]['name'],
            'robot_idx': robot_idx,
            'has_dynamic_weight': finger_name in self.dynamic_weight_configs,
            'has_extended_mapping': self.extended_mapping_enabled.get(finger_name, False),
            'scale_factor': self.scale_factors.get(finger_name, 1.0),
            'max_angle': max_angle
        }
        
        # 如果有open和opose状态，显示相关信息
        if 'open' in self.robot_states and 'opose' in self.robot_states:
            open_angle = self.robot_states['open'][robot_idx]
            opose_angle = self.robot_states['opose'][robot_idx]
            info.update({
                'open_angle': open_angle,
                'opose_angle': opose_angle,
                'available_extension': max_angle - opose_angle
            })
        
        return info
