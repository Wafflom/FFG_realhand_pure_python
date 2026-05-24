"""
RealForce L20 手型映射模块 - 纯 Python 版本
"""

import numpy as np
import copy
from ..realhand_core import RealHandCore as HandCore
from ..realforce_config.l20_config import (
    FINGER_CONFIGS, MAPPING_ORDER, MULTI_SEGMENT_CONFIG,
    ROBOT_ORIGINAL_LEFT, ROBOT_ORIGINAL_RIGHT,
    ROBOT_FIST_LEFT, ROBOT_FIST_RIGHT,
    ROBOT_OPOSE_LEFT, ROBOT_OPOSE_RIGHT,
    MOTOR_CONSTRAINTS
)
from typing import List
from ..realhand_core_ex import DynamicWeightMultiStateLinearMapper,MultiStateLinearMapper


class RightHand:
    def __init__(self, handcore: HandCore, length=20, is_debug: bool = False):
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

        # self.robot_fist[0] = -0.2 # 拇指旋转锁死在最大0.2，高于0.2的属于无用区间
        # self.robot_fist[1] = 1.4 # 拇指侧摆锁死在最大1.2，高于1.2的属于无用区间

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
        """
        初始化映射器

        将三种人手标定数据和三种机械手标定数据加载到映射器中
        分别是original,opose,fist

        人手是glove_前缀,机械手是robot_前缀
        """
        # 侧摆部分预处理
        for i in [5, 9, 13, 17]:
            self.calibrationoriginal[i] = self.calibrationopose[i] + 0.1
            self.calibrationfistpose[i] = self.calibrationopose[i] - 0.1

        glove_original = self._to_list(self.calibrationoriginal)
        glove_fist = self._to_list(self.calibrationfistpose)
        glove_opose = self._to_list(self.calibrationopose)
        
        self.multi_state_mapper.add_state('original', glove_original, self.robot_original)
        self.multi_state_mapper.add_state('opose', glove_opose, self.robot_opose)
        self.multi_state_mapper.add_state('fist', glove_fist, self.robot_fist)

        self.multi_state_mapper.set_state_order(MULTI_SEGMENT_CONFIG['states'])

        state_info = self.multi_state_mapper.get_state_info()

    def _to_list(self, data):
        """转换为列表"""
        if hasattr(data, 'tolist'):
            return data.tolist()
        elif isinstance(data, np.ndarray):
            print(111)
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

    # V2.8.0 本函数作废
    # def _linear_map_diff(self, current_diff, fist_diff, extend_ratio=1.2):
    #     """
    #     基于差值的线性映射到0-255

    #     注意: 传入joint_update的是差值 (当前值 - 张开值)

    #     参数:
    #         current_diff: 当前传感器差值 (当前值 - 张开值)
    #         fist_diff: 握拳时的差值 (握拳值 - 张开值)
    #         extend_ratio: 缩放比例，>1.0 使映射更容易到达0/255边界

    #     映射逻辑：
    #         - 差值为0（张开）→ 255
    #         - 差值为fist_diff（握拳）→ 0
    #     """
    #     if abs(fist_diff) < 0.01:
    #         return 128  # 变化太小，返回中值

    #     # 缩小fist_diff使得更容易到达0边界
    #     effective_fist_diff = fist_diff / extend_ratio

    #     # 计算比例: 差值0→比例0, 差值fist_diff→比例1
    #     ratio = current_diff / effective_fist_diff
    #     ratio = max(0.0, min(1.0, ratio))  # 限制在0-1之间

    #     # 映射: 比例0→255, 比例1→0
    #     return int((1 - ratio) * 255)

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
            qpos[16] = self.g_jointpositions_arc[0] = arc_value[0] 
            qpos[17] = self.g_jointpositions_arc[1] = arc_value[1]
            qpos[18] = self.g_jointpositions_arc[2] = arc_value[2]
            qpos[19] = self.g_jointpositions_arc[3] = arc_value[3]
            
            qpos[0] = self.g_jointpositions_arc[5] = arc_value[5]
            qpos[1] = self.g_jointpositions_arc[6] = arc_value[6]

            qpos[2] = self.g_jointpositions_arc[7] = arc_value[7]
            qpos[3] = self.g_jointpositions_arc[8] = arc_value[8]

            qpos[4] = self.g_jointpositions_arc[17] = arc_value[17]
            qpos[5] = self.g_jointpositions_arc[18] = arc_value[18]
            qpos[6] = self.g_jointpositions_arc[19] = arc_value[19]
            qpos[7] = self.g_jointpositions_arc[4] = arc_value[20]

            qpos[8] = self.g_jointpositions_arc[9] = arc_value[9]
            qpos[9] = self.g_jointpositions_arc[10] = arc_value[10]
            qpos[10] = self.g_jointpositions_arc[11] = arc_value[11]
            qpos[11] = self.g_jointpositions_arc[12] = arc_value[12]

            qpos[12] = self.g_jointpositions_arc[13] = arc_value[13]
            qpos[13] = self.g_jointpositions_arc[14] = arc_value[14]
            qpos[14] = self.g_jointpositions_arc[15] = arc_value[15]
            qpos[15] = self.g_jointpositions_arc[16] = arc_value[16]              
        # ========== 没有标定数据时使用手动映射 ==========
        else:
            # 拇指处理 (与O6相同)
            qpos[20] = joint_arc[4] * 2.2   # 拇指弯曲
            qpos[17] = joint_arc[2] * -2.5  # 拇指侧摆
            # 四指处理
            qpos[1] = joint_arc[6] * 0.1 + joint_arc[8] * 0.7
            qpos[9] = joint_arc[10] * 0.1 + joint_arc[12] * 0.7
            qpos[13] = joint_arc[14] * 0.1 + joint_arc[16] * 0.7
            qpos[5] = joint_arc[18] * 0.1 + joint_arc[20] * 0.7
            # self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)

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


class LeftHand:
    def __init__(self, handcore: HandCore, length=20, is_debug: bool = False):
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

        self.robot_fist[0] = 0.3 # 拇指旋转锁死在最大0.2，高于0.2的属于无用区间
        # self.robot_fist[1] = 1.5 # 拇指侧摆锁死在最大1.2，高于1.2的属于无用区间

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
        """
        初始化映射器

        将三种人手标定数据和三种机械手标定数据加载到映射器中
        分别是original,opose,fist

        人手是glove_前缀,机械手是robot_前缀
        """
        # 侧摆部分预处理
        for i in [5, 9, 13, 17]:
            self.calibrationoriginal[i] = self.calibrationopose[i] - 0.1
            self.calibrationfistpose[i] = self.calibrationopose[i] + 0.1

        glove_original = self._to_list(self.calibrationoriginal)
        glove_fist = self._to_list(self.calibrationfistpose)
        glove_opose = self._to_list(self.calibrationopose)
        
        self.multi_state_mapper.add_state('original', glove_original, self.robot_original)
        self.multi_state_mapper.add_state('opose', glove_opose, self.robot_opose)
        self.multi_state_mapper.add_state('fist', glove_fist, self.robot_fist)

        self.multi_state_mapper.set_state_order(MULTI_SEGMENT_CONFIG['states'])

        state_info = self.multi_state_mapper.get_state_info()

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

    # V2.8.0 本函数作废
    # def _linear_map_diff(self, current_diff, fist_diff, extend_ratio=1.2):
    #     """
    #     基于差值的线性映射到0-255

    #     注意: 传入joint_update的是差值 (当前值 - 张开值)

    #     参数:
    #         current_diff: 当前传感器差值 (当前值 - 张开值)
    #         fist_diff: 握拳时的差值 (握拳值 - 张开值)
    #         extend_ratio: 缩放比例，>1.0 使映射更容易到达0/255边界

    #     映射逻辑：
    #         - 差值为0（张开）→ 255
    #         - 差值为fist_diff（握拳）→ 0
    #     """
    #     if abs(fist_diff) < 0.01:
    #         return 128  # 变化太小，返回中值
        
    #     # 缩小fist_diff使得更容易到达0边界
    #     effective_fist_diff = fist_diff / extend_ratio
        
    #     # 计算比例: 差值0→比例0, 差值fist_diff→比例1
    #     ratio = current_diff / effective_fist_diff
    #     ratio = max(0.0, min(1.0, ratio))  # 限制在0-1之间
        
    #     # 映射: 比例0→255, 比例1→0
    #     return int((1 - ratio) * 255)

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
            # arc_value = ROBOT_OPOSE_LEFT
            qpos[16] = self.g_jointpositions_arc[0] = arc_value[0] 
            qpos[17] = self.g_jointpositions_arc[1] = arc_value[1]
            qpos[18] = self.g_jointpositions_arc[2] = arc_value[2]
            qpos[19] = self.g_jointpositions_arc[3] = arc_value[3]
            
            qpos[0] = self.g_jointpositions_arc[5] = arc_value[5] * 3
            qpos[1] = self.g_jointpositions_arc[6] = arc_value[6]
            qpos[2] = self.g_jointpositions_arc[7] = arc_value[7]
            qpos[3] = self.g_jointpositions_arc[8] = arc_value[8]

            qpos[4] = self.g_jointpositions_arc[17] = arc_value[17]
            qpos[5] = self.g_jointpositions_arc[18] = arc_value[18]
            qpos[6] = self.g_jointpositions_arc[19] = arc_value[19]
            qpos[7] = self.g_jointpositions_arc[4] = arc_value[20]

            qpos[8] = self.g_jointpositions_arc[9] = arc_value[9]
            qpos[9] = self.g_jointpositions_arc[10] = arc_value[10]
            qpos[10] = self.g_jointpositions_arc[11] = arc_value[11]
            qpos[11] = self.g_jointpositions_arc[12] = arc_value[12]

            qpos[12] = self.g_jointpositions_arc[13] = arc_value[13]
            qpos[13] = self.g_jointpositions_arc[14] = arc_value[14]
            qpos[14] = self.g_jointpositions_arc[15] = arc_value[15]
            qpos[15] = self.g_jointpositions_arc[16] = arc_value[16]              
        # ========== 没有标定数据时使用手动映射 ==========
        else:
            # 拇指处理 (与O6相同)
            qpos[20] = joint_arc[4] * 2.2   # 拇指弯曲
            qpos[17] = joint_arc[2] * -2.5  # 拇指侧摆
            # 四指处理
            qpos[1] = joint_arc[6] * 0.1 + joint_arc[8] * 0.7
            qpos[9] = joint_arc[10] * 0.1 + joint_arc[12] * 0.7
            qpos[13] = joint_arc[14] * 0.1 + joint_arc[16] * 0.7
            qpos[5] = joint_arc[18] * 0.1 + joint_arc[20] * 0.7
            # self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)
        
        # ========== 应用电机约束 ==========
        self.g_jointpositions = self.handcore.trans_to_motor_left(qpos)
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