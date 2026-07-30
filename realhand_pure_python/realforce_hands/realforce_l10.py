"""
RealForce L10 手型映射模块 - 纯 Python 版本
"""
import numpy as np
import copy
from ..realhand_core import RealHandCore as HandCore
from ..realforce_config.l10_config import FINGER_CONFIGS, MAPPING_ORDER, ROBOT_OPOSE_RIGHT, ROBOT_OPOSE_LEFT, ROBOT_ORIGINAL_RIGHT, ROBOT_ORIGINAL_LEFT, ROBOT_FIST_RIGHT, ROBOT_FIST_LEFT, MULTI_SEGMENT_CONFIG_FROZEN, MOTOR_CONSTRAINTS
from ..realhand_core_ex import DynamicWeightMultiStateLinearMapper

def _resolve_version_config(configs: dict, version: str) -> dict:
    """
    解析版本配置，将字典格式的 weights/reverse_motion 转换为具体值
    """
    resolved = copy.deepcopy(configs)
    for finger_name, config in resolved.items():
        if 'weights' in config and isinstance(config['weights'], dict):
            config['weights'] = config['weights'].get(version, config['weights'].get('v2', [0.5, 0, 0.5]))
        if 'reverse_motion' in config and isinstance(config['reverse_motion'], dict):
            config['reverse_motion'] = config['reverse_motion'].get(version, config['reverse_motion'].get('v2', False))
    return resolved

class RightHand:
    def __init__(self, handcore: HandCore, length=10, is_debug: bool = False):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions_arc = [0] * length
        self.g_jointvelocity_arc = [0] * length
        self.calibrationoriginal = None
        self.calibrationfistpose = None
        self.calibrationopose = None
        self.glove_version = 'v2'
        
        self.smooth_enabled = True
        self.smooth_alpha = 0.5
        self.smooth_positions = [255.0] * length
        self.max_step = 20

        self.robot_original = ROBOT_ORIGINAL_RIGHT
        self.robot_opose = ROBOT_OPOSE_RIGHT
        self.robot_fist = ROBOT_FIST_RIGHT

        finger_configs = _resolve_version_config(FINGER_CONFIGS, self.glove_version)
        self.multi_state_mapper = DynamicWeightMultiStateLinearMapper(finger_configs, MAPPING_ORDER, is_debug=is_debug)

        for config_name, config in FINGER_CONFIGS.items():
            if config.get('dynamic_weight'):
                self.multi_state_mapper.set_dynamic_weight_config(config_name, config['dynamic_weight'])

        self.motor_constraints = MOTOR_CONSTRAINTS['right']
    
    def _apply_motor_constraints(self):
        for i, constraint in enumerate(self.motor_constraints):
            if constraint.get('enabled', False):
                min_val = constraint.get('min', 0)
                max_val = constraint.get('max', 255)
                self.g_jointpositions[i] = int(max(min_val, min(max_val, self.g_jointpositions[i])))
    
    def set_glove_version(self, version: str):
        if not version:
            return
        
        major_version = version.split('.')[0]
        version_key = f'v{major_version}'
        
        if version_key == self.glove_version:
            return
            
        self.glove_version = version_key
        
        for finger_name, config in FINGER_CONFIGS.items():
            if 'weights' in config and isinstance(config['weights'], dict):
                if version_key in config['weights']:
                    self.multi_state_mapper.finger_configs[finger_name]['weights'] = config['weights'][version_key]
            
            if 'reverse_motion' in config and isinstance(config['reverse_motion'], dict):
                if version_key in config['reverse_motion']:
                    self.multi_state_mapper.finger_configs[finger_name]['reverse_motion'] = config['reverse_motion'][version_key]

    def initialize_mapper(self) -> bool:
        glove_original = self._to_list(self.calibrationoriginal)
        glove_fist = self._to_list(self.calibrationfistpose)
        glove_opose = self._to_list(self.calibrationopose)
        
        self.multi_state_mapper.add_state('original', glove_original, self.robot_original)
        self.multi_state_mapper.add_state('opose', glove_opose, self.robot_opose)
        self.multi_state_mapper.add_state('fist', glove_fist, self.robot_fist)
        self.multi_state_mapper.set_state_order(list(MULTI_SEGMENT_CONFIG_FROZEN))
    
    def _to_list(self, data):
        """转换为列表"""
        if hasattr(data, 'tolist'):
            return data.tolist()
        elif isinstance(data, np.ndarray):
            return data.tolist()
        else:
            return list(data)
    
    def _apply_smooth(self, raw_positions):
        """
        对电机输出应用平滑滤波，防止跳变
        
        使用指数移动平均(EMA) + 最大步长限制
        """
        if not self.smooth_enabled:
            return raw_positions
        
        smoothed = []
        for i, raw in enumerate(raw_positions):
            # 指数移动平均
            target = self.smooth_alpha * raw + (1 - self.smooth_alpha) * self.smooth_positions[i]
            
            # 最大步长限制，防止大幅跳变
            diff = target - self.smooth_positions[i]
            if abs(diff) > self.max_step:
                target = self.smooth_positions[i] + (self.max_step if diff > 0 else -self.max_step)
            
            self.smooth_positions[i] = target
            smoothed.append(int(round(target)))
        
        return smoothed

    def joint_update(self, joint_arc):
        """
        右手映射 - 基于标定数据和预期机械手动作的映射器完成
        """
        qpos = np.zeros(25)
        # ========== 使用映射器进行精确映射 ==========
        if self.calibrationoriginal is not None and self.calibrationfistpose is not None and self.calibrationopose is not None:
        
            arc_value = self.multi_state_mapper.map_glove_to_robot(joint_arc)
            # arc_value = ROBOT_OPOSE_RIGHT
            qpos[20] = self.g_jointpositions_arc[0] = arc_value[2]
            qpos[17] = self.g_jointpositions_arc[1] = arc_value[1]
            qpos[1] = self.g_jointpositions_arc[2] = arc_value[6]
            qpos[9] = self.g_jointpositions_arc[3] = arc_value[9]
            qpos[13] = self.g_jointpositions_arc[4] = arc_value[13]
            qpos[5] = self.g_jointpositions_arc[5] = arc_value[17]
            qpos[0] = self.g_jointpositions_arc[6] = arc_value[5]
            qpos[12] = self.g_jointpositions_arc[7] = arc_value[12]
            qpos[4] = self.g_jointpositions_arc[8] = arc_value[16]
            qpos[16] = self.g_jointpositions_arc[9] = arc_value[0]
        else:
            # 拇指处理
            qpos[16] = joint_arc[1] * 1.2
            qpos[17] = joint_arc[0] * -2 + joint_arc[1] * 1.2
            qpos[20] = joint_arc[4] * 0.5 + joint_arc[2] * 0.8
    
            qpos[1] = joint_arc[6] * 0.1 + joint_arc[8] * 0.7
            qpos[9] = joint_arc[10] * 0.1 + joint_arc[12] * 0.7
            qpos[13] = joint_arc[14] * 0.1 + joint_arc[16] * 0.7
            qpos[4] = joint_arc[18] * 0.1 + joint_arc[20] * 0.7
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)
        
        self._apply_motor_constraints()

    def speed_update(self):
        # The original adaptive stop/slow/fast velocity state machine always
        # ended by overwriting its result with 255, so only that effective
        # behavior is kept. The full logic is in git history if ever needed.
        for i in range(len(self.g_jointpositions)):
            self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = 255
            self.last_jointpositions[i] = self.g_jointpositions[i]

class LeftHand:
    def __init__(self, handcore: HandCore, length=10, is_debug: bool = False):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions_arc = [0] * length
        self.g_jointvelocity_arc = [0] * length
        self.calibrationoriginal = None
        self.calibrationfistpose = None
        self.calibrationopose = None
        self.glove_version = 'v2'
        
        self.smooth_enabled = True
        self.smooth_alpha = 0.5
        self.smooth_positions = [255.0] * length
        self.max_step = 20

        self.robot_original = ROBOT_ORIGINAL_LEFT
        self.robot_opose = ROBOT_OPOSE_LEFT
        self.robot_fist = ROBOT_FIST_LEFT

        finger_configs = _resolve_version_config(FINGER_CONFIGS, self.glove_version)
        self.multi_state_mapper = DynamicWeightMultiStateLinearMapper(finger_configs, MAPPING_ORDER, is_debug=is_debug)

        for config_name, config in FINGER_CONFIGS.items():
            if config.get('dynamic_weight'):
                self.multi_state_mapper.set_dynamic_weight_config(config_name, config['dynamic_weight'])

        self.motor_constraints = MOTOR_CONSTRAINTS['left']

    def _apply_motor_constraints(self):
        for i, constraint in enumerate(self.motor_constraints):
            if constraint.get('enabled', False):
                min_val = constraint.get('min', 0)
                max_val = constraint.get('max', 255)
                self.g_jointpositions[i] = int(max(min_val, min(max_val, self.g_jointpositions[i])))

    def set_glove_version(self, version: str):
        if not version:
            return
        
        major_version = version.split('.')[0]
        version_key = f'v{major_version}'
        
        if version_key == self.glove_version:
            return
            
        self.glove_version = version_key
        
        for finger_name, config in FINGER_CONFIGS.items():
            if 'weights' in config and isinstance(config['weights'], dict):
                if version_key in config['weights']:
                    self.multi_state_mapper.finger_configs[finger_name]['weights'] = config['weights'][version_key]
            
            if 'reverse_motion' in config and isinstance(config['reverse_motion'], dict):
                if version_key in config['reverse_motion']:
                    self.multi_state_mapper.finger_configs[finger_name]['reverse_motion'] = config['reverse_motion'][version_key]

    def initialize_mapper(self) -> bool:
        glove_original = self._to_list(self.calibrationoriginal)
        glove_fist = self._to_list(self.calibrationfistpose)
        glove_opose = self._to_list(self.calibrationopose)
        
        self.multi_state_mapper.add_state('original', glove_original, self.robot_original)
        self.multi_state_mapper.add_state('opose', glove_opose, self.robot_opose)
        self.multi_state_mapper.add_state('fist', glove_fist, self.robot_fist)
        self.multi_state_mapper.set_state_order(list(MULTI_SEGMENT_CONFIG_FROZEN))
    
    def _to_list(self, data):
        """转换为列表"""
        if hasattr(data, 'tolist'):
            return data.tolist()
        elif isinstance(data, np.ndarray):
            return data.tolist()
        else:
            return list(data)
    
    def _apply_smooth(self, raw_positions):
        """
        对电机输出应用平滑滤波，防止跳变
        
        使用指数移动平均(EMA) + 最大步长限制
        """
        if not self.smooth_enabled:
            return raw_positions
        
        smoothed = []
        for i, raw in enumerate(raw_positions):
            # 指数移动平均
            target = self.smooth_alpha * raw + (1 - self.smooth_alpha) * self.smooth_positions[i]
            
            # 最大步长限制，防止大幅跳变
            diff = target - self.smooth_positions[i]
            if abs(diff) > self.max_step:
                target = self.smooth_positions[i] + (self.max_step if diff > 0 else -self.max_step)
            
            self.smooth_positions[i] = target
            smoothed.append(int(round(target)))
        
        return smoothed

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        if self.calibrationoriginal is not None and self.calibrationfistpose is not None and self.calibrationopose is not None:
            arc_value = self.multi_state_mapper.map_glove_to_robot(joint_arc)
            qpos[20] = self.g_jointpositions_arc[0] = arc_value[2]
            qpos[17] = self.g_jointpositions_arc[1] = arc_value[1]
            qpos[1] = self.g_jointpositions_arc[2] = arc_value[6]
            qpos[9] = self.g_jointpositions_arc[3] = arc_value[9]
            qpos[13] = self.g_jointpositions_arc[4] = arc_value[13]
            qpos[5] = self.g_jointpositions_arc[5] = arc_value[17]
            qpos[0] = self.g_jointpositions_arc[6] = arc_value[5]
            qpos[12] = self.g_jointpositions_arc[7] = arc_value[12]
            qpos[4] = self.g_jointpositions_arc[8] = arc_value[16]
            qpos[16] = self.g_jointpositions_arc[9] = arc_value[0]
        else:
            qpos[16] = joint_arc[1] * 1.2
            qpos[17] = joint_arc[0] * -2 + joint_arc[1] * 1.2
            qpos[20] = joint_arc[4] * 0.5 + joint_arc[2] * 0.8
    
            qpos[1] = joint_arc[6] * 0.1 + joint_arc[8] * 0.7
            qpos[9] = joint_arc[10] * 0.1 + joint_arc[12] * 0.7
            qpos[13] = joint_arc[14] * 0.1 + joint_arc[16] * 0.7
            qpos[5] = joint_arc[18] * 0.1 + joint_arc[20] * 0.7
        self.g_jointpositions = self.handcore.trans_to_motor_left(qpos)
        self._apply_motor_constraints()


    def speed_update(self):
        # The original adaptive stop/slow/fast velocity state machine always
        # ended by overwriting its result with 255, so only that effective
        # behavior is kept. The full logic is in git history if ever needed.
        for i in range(len(self.g_jointpositions)):
            self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = 255
            self.last_jointpositions[i] = self.g_jointpositions[i]