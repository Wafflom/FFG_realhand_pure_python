# 手指配置常量
FINGER_CONFIGS = {
    'thumb_abduction': {
        'name': '拇指侧摆',
        'joints': [0, 1, 2],
        'weights': {
            'v1': [0, 0, 1],
            'v2': [0, 1, 0]
        },
        'robot_idx': 0,
        'type': 'thumb',
        'reverse_motion': False,
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.2,
            'extended_exp_factor': 10
        }
    },
    'thumb_root_flexion': {
        'name': '拇指弯曲',
        'joints': [2, 3, 4],
        'weights': {
            'v1': [0, 0, 1],
            'v2': [0.2, 0, 0.8]
        },
        'robot_idx': 1,
        'type': 'thumb',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.0,
            'extended_exp_factor': 10
        }
    },
    'index_root_flexion': {
        'name': '食指',
        'joints': [6, 7, 8],
        'weights': {
            'v1': [1.0, 0.0, 0.0],
            'v2': [1.0, 0.0, 0.0]
        },
        'robot_idx': 3,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.0,
            'extended_exp_factor': 10
        }
    },
    'middle_root_flexion': {
        'name': '中指',
        'joints': [10, 11, 12],
        'weights': {
            'v1': [1.0, 0.0, 0.0],
            'v2': [1.0, 0.0, 0.0]
        },
        'robot_idx': 5,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.0,
            'extended_exp_factor': 10
        }
    },
    'ring_root_flexion': {
        'name': '无名指',
        'joints': [14, 15, 16],
        'weights': {
            'v1': [1.0, 0.0, 0.0],
            'v2': [1.0, 0.0, 0.0]
        },
        'robot_idx': 7,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.0,
            'extended_exp_factor': 10
        }
    },
    'pinky_root_flexion': {
        'name': '小指',
        'joints': [18, 19, 20],
        'weights': {
            'v1': [1.0, 0.0, 0.0],
            'v2': [1.0, 0.0, 0.0]
        },
        'robot_idx': 9,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.0,
            'extended_exp_factor': 10
        }
    }
}

MAPPING_ORDER = [
    'thumb_abduction', 'thumb_root_flexion',
    'index_root_flexion', 'middle_root_flexion', 'ring_root_flexion', 'pinky_root_flexion'
]

MULTI_SEGMENT_CONFIG = {
    'states': [
        'original', 
        # 'opose',
        'fist'
        ],
    'state_names': {
        'original': '张手',
        # 'opose': 'O手势',
        'fist': '握拳'
    }
}
MULTI_SEGMENT_CONFIG_FROZEN = tuple(MULTI_SEGMENT_CONFIG['states'])

ROBOT_OPOSE_LEFT = [
    1.1, 0.33, 0.0, 0.84, 0.0, 0.84, 0.0, 0.84, 0.0, 0.84, 0.0
]

ROBOT_OPOSE_RIGHT = [
    1.1, 0.33, 0.0, 0.84, 0.0, 0.84, 0.0, 0.84, 0.0, 0.84, 0.0
]

ROBOT_ORIGINAL_LEFT = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
]

ROBOT_ORIGINAL_RIGHT = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
]

ROBOT_FIST_LEFT = [
    1.54, 0.52, 0.96, 1.57, 1.4, 1.57, 1.4, 1.57, 1.4, 1.57, 1.4
]

ROBOT_FIST_RIGHT = [
    1.54, 0.52, 0.96, 1.57, 1.4, 1.57, 1.4, 1.57, 1.4, 1.57, 1.4
]

PLOTGUI_ROBOT_ID = [
    0, 1, 2
]

# 电机输出约束配置 (6电机)
MOTOR_CONSTRAINTS = {
    'left': [
        {'min': 0, 'max': 255, 'enabled': False},   # motor 0: 拇指侧摆
        {'min': 0, 'max': 255, 'enabled': False},   # motor 1: 拇指根部
        {'min': 0, 'max': 255, 'enabled': False},   # motor 2: 食指
        {'min': 0, 'max': 255, 'enabled': False},   # motor 3: 中指
        {'min': 0, 'max': 255, 'enabled': False},   # motor 4: 无名指
        {'min': 0, 'max': 255, 'enabled': False},   # motor 5: 小指
    ],
    'right': [
        {'min': 0, 'max': 255, 'enabled': False},   # motor 0: 拇指侧摆
        {'min': 0, 'max': 255, 'enabled': False},   # motor 1: 拇指根部
        {'min': 0, 'max': 255, 'enabled': False},   # motor 2: 食指
        {'min': 0, 'max': 255, 'enabled': False},   # motor 3: 中指
        {'min': 0, 'max': 255, 'enabled': False},   # motor 4: 无名指
        {'min': 0, 'max': 255, 'enabled': False},   # motor 5: 小指
    ]
}