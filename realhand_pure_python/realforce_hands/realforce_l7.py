"""
RealForce L7 手型映射模块 - 纯 Python 版本
"""
import numpy as np
import copy
from ..realhand_core import RealHandCore as HandCore
from ..realforce_config.o7_config import FINGER_CONFIGS, MAPPING_ORDER, ROBOT_OPOSE_RIGHT, ROBOT_OPOSE_LEFT, ROBOT_ORIGINAL_RIGHT, ROBOT_ORIGINAL_LEFT, ROBOT_FIST_RIGHT, ROBOT_FIST_LEFT, MULTI_SEGMENT_CONFIG, MULTI_SEGMENT_CONFIG_FROZEN, MOTOR_CONSTRAINTS
from typing import List
from ..realhand_core_ex import DynamicWeightMultiStateLinearMapper,MultiStateLinearMapper


class RightHand:
    def __init__(self, handcore: HandCore, length=7, is_debug: bool = False):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions_arc = [0] * length
        self.g_jointvelocity_arc = [0] * length
        self.handstate = [0] * length
        self.calibrationoriginal = None    # 五指张开标定值 (对应255)
        self.calibrationfistpose = None    # 握拳标定值 (对应0)
        self.calibrationopose = None       # O型标定值 (对应中间值)
        self.glove_version = 'v2'
        
        # ========== 平滑滤波参数 ==========
        self.smooth_enabled = True
        self.smooth_alpha = 0.5  # 平滑系数：越小越平滑，范围 0.05-0.3
        self.smooth_positions = [255.0] * length  # 平滑后的位置（浮点）
        self.max_step = 20  # 每帧最大变化量，防止跳变

        # 目标机械手预设姿势，数值从URDF获取数据集，
        # 张开手的时候对应最小角度，
        # 握拳的时候对应最大角度
        # O型手势的时候，用工具驱动URDF去驱动目标机械手达到期望姿势，也可以调整这些参数使得实物更加达到期望角度
        # 其他手势也类似，也可以增加多个手势来实现多模态的映射器（后期陆续开发）
        self.robot_original = ROBOT_ORIGINAL_RIGHT
        self.robot_opose = ROBOT_OPOSE_RIGHT
        self.robot_fist = ROBOT_FIST_RIGHT
        self.robot_fist[0] = 0.8

        # 这里可以额外对self.robot_fist的非期望值进行修正，
        # 由于机械手达到最大值，存在非期望值的区域，在这里可以进行修正，
        # 同样也需要URDF驱动工具包去做这个事情


        # 映射器（v2.8.0专属），具体介绍参考l6_config.py文件
        self.multi_state_mapper = DynamicWeightMultiStateLinearMapper(FINGER_CONFIGS, MAPPING_ORDER, is_debug=is_debug)

        # 设置动态权重配置（v2.8.2新增）
        for config_name, config in FINGER_CONFIGS.items():
            if config.get('dynamic_weight'):
                self.multi_state_mapper.set_dynamic_weight_config(config_name, config['dynamic_weight'])

        # 电机输出约束
        self.motor_constraints = MOTOR_CONSTRAINTS['right']

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
        """初始化映射器"""
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

    def _apply_motor_constraints(self, positions):
        """应用电机输出约束"""
        if not hasattr(self, 'motor_constraints') or self.motor_constraints is None:
            return positions
        result = []
        for i, pos in enumerate(positions):
            if i < len(self.motor_constraints) and self.motor_constraints[i].get('enabled', False):
                result.append(max(self.motor_constraints[i]['min'], min(pos, self.motor_constraints[i]['max'])))
            else:
                result.append(pos)
        return result

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
            qpos[17] = self.g_jointpositions_arc[1] = arc_value[1]
            qpos[20] = self.g_jointpositions_arc[0] = arc_value[2]
            qpos[16] = self.g_jointpositions_arc[6] = arc_value[0]

            qpos[1] = self.g_jointpositions_arc[2] = arc_value[3]

            qpos[9] = self.g_jointpositions_arc[3] = arc_value[4]

            qpos[13] = self.g_jointpositions_arc[4] = arc_value[5]

            qpos[5] = self.g_jointpositions_arc[5] = arc_value[6]
        else:
            # 拇指处理
            qpos[16] = joint_arc[1] * 1.2
            qpos[17] = joint_arc[0] * -2 + joint_arc[1] * 1.2
            qpos[20] = joint_arc[4] * 0.5 + joint_arc[2] * 0.8
    
            qpos[1] = joint_arc[6] * 0.1 + joint_arc[8] * 0.7
            qpos[9] = joint_arc[10] * 0.1 + joint_arc[12] * 0.7
            qpos[13] = joint_arc[14] * 0.1 + joint_arc[16] * 0.7
            qpos[5] = joint_arc[18] * 0.1 + joint_arc[20] * 0.7
        # ========== 应用电机约束 ==========
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)
        self.g_jointpositions = self._apply_motor_constraints(self.g_jointpositions)
        # ========== 应用平滑滤波 ==========
        self.g_jointpositions = self._apply_smooth(self.g_jointpositions)

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            position_derict = 1 if self.g_jointpositions[i] - lastpos > 0 else -1
            slow_limit = 4
            fast_limit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.7)
            min_vel = int(self.last_jointvelocity[i] * 0.5)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 5 + 30
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 50
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                    target_vel = position_error * 5 + 100
                else:
                    target_vel = position_error * 5 + 100
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 90
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 5 + 60
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 5 + 40
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            self.g_jointvelocity[i] = int(target_vel * 1)
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]


# LeftHand 类类似修正
class LeftHand:
    def __init__(self, handcore: HandCore, length=7, is_debug: bool = False):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions_arc = [0] * length
        self.g_jointvelocity_arc = [0] * length
        self.handstate = [0] * length
        self.calibrationoriginal = None    # 五指张开标定值 (对应255)
        self.calibrationfistpose = None    # 握拳标定值 (对应0)
        self.calibrationopose = None       # O型标定值 (对应中间值)
        self.glove_version = 'v2'

        # ========== 平滑滤波参数 ==========
        self.smooth_enabled = True
        self.smooth_alpha = 0.5  # 平滑系数：越小越平滑，范围 0.05-0.3
        self.smooth_positions = [255.0] * length  # 平滑后的位置（浮点）
        self.max_step = 20  # 每帧最大变化量，防止跳变

        # 目标机械手预设姿势，数值从URDF获取数据集，
        # 张开手的时候对应最小角度，
        # 握拳的时候对应最大角度
        # O型手势的时候，用工具驱动URDF去驱动目标机械手达到期望姿势，也可以调整这些参数使得实物更加达到期望角度
        # 其他手势也类似，也可以增加多个手势来实现多模态的映射器（后期陆续开发）
        self.robot_original = ROBOT_ORIGINAL_LEFT
        self.robot_opose = ROBOT_OPOSE_LEFT
        self.robot_fist = ROBOT_FIST_LEFT
        self.robot_fist[0] = 0.8

        # 这里可以额外对self.robot_fist的非期望值进行修正，
        # 由于机械手达到最大值，存在非期望值的区域，在这里可以进行修正，
        # 同样也需要URDF驱动工具包去做这个事情


        # 映射器（v2.8.0专属），具体介绍参考l6_config.py文件
        self.multi_state_mapper = DynamicWeightMultiStateLinearMapper(FINGER_CONFIGS, MAPPING_ORDER, is_debug=is_debug)

        # 设置动态权重配置（v2.8.2新增）
        for config_name, config in FINGER_CONFIGS.items():
            if config.get('dynamic_weight'):
                self.multi_state_mapper.set_dynamic_weight_config(config_name, config['dynamic_weight'])

        # 电机输出约束
        self.motor_constraints = MOTOR_CONSTRAINTS['left']

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
        """初始化映射器"""
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

    def _apply_motor_constraints(self, positions):
        """应用电机输出约束"""
        if not hasattr(self, 'motor_constraints') or self.motor_constraints is None:
            return positions
        result = []
        for i, pos in enumerate(positions):
            if i < len(self.motor_constraints) and self.motor_constraints[i].get('enabled', False):
                result.append(max(self.motor_constraints[i]['min'], min(pos, self.motor_constraints[i]['max'])))
            else:
                result.append(pos)
        return result

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
            qpos[17] = self.g_jointpositions_arc[1] = arc_value[1]
            qpos[20] = self.g_jointpositions_arc[0] = arc_value[2]
            qpos[16] = self.g_jointpositions_arc[6] = arc_value[0]

            qpos[1] = self.g_jointpositions_arc[2] = arc_value[3]

            qpos[9] = self.g_jointpositions_arc[3] = arc_value[4]

            qpos[13] = self.g_jointpositions_arc[4] = arc_value[5]

            qpos[5] = self.g_jointpositions_arc[5] = arc_value[6]
        else:
            # 拇指处理
            qpos[16] = joint_arc[1] * 1.2
            qpos[17] = joint_arc[0] * -2 + joint_arc[1] * 1.2
            qpos[20] = joint_arc[4] * 0.5 + joint_arc[2] * 0.8
    
            qpos[1] = joint_arc[6] * 0.1 + joint_arc[8] * 0.7
            qpos[9] = joint_arc[10] * 0.1 + joint_arc[12] * 0.7
            qpos[13] = joint_arc[14] * 0.1 + joint_arc[16] * 0.7
            qpos[5] = joint_arc[18] * 0.1 + joint_arc[20] * 0.7
        # ========== 应用电机约束 ==========
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)
        self.g_jointpositions = self._apply_motor_constraints(self.g_jointpositions)
        # ========== 应用平滑滤波 ==========
        self.g_jointpositions = self._apply_smooth(self.g_jointpositions)

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            position_derict = 1 if self.g_jointpositions[i] - lastpos > 0 else -1
            slow_limit = 4
            fast_limit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.7)
            min_vel = int(self.last_jointvelocity[i] * 0.5)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 5 + 30
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 50
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                    target_vel = position_error * 5 + 100
                else:
                    target_vel = position_error * 5 + 100
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 90
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 5 + 60
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 5 + 40
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            self.g_jointvelocity[i] = int(target_vel * 1)
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]

