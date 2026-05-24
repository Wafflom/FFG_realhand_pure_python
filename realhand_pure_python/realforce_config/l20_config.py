# 手指配置常量
FINGER_CONFIGS = {
    'thumb_rotate': {
        'name': '拇指旋转',
        'joints': [1, 2],
        'weights': {
            'v1': [1, 0],
            'v2': [1, 0]
        },
        'robot_idx': 0,
        'type': 'thumb',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.0,
            'extended_exp_factor': 1.5
        }
    },
    'thumb_abduction': {
        'name': '拇指侧摆',
        'joints': [0, 1, 2],
        'weights': {
            'v1': [0.7, 0.3, 0],
            'v2': [0.7, 0.3, 0]
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
            'scale_factor': 1.2,
            'extended_exp_factor': 5
        }
    },
    'thumb_root_flexion': {
        'name': '拇指根部弯曲',
        'joints': [2, 3, 4],
        'weights': {
            'v1': [1, 0, 0],
            'v2': [1, 0, 0]
        },
        'robot_idx': 2,
        'type': 'thumb',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': {
            'trigger_finger': 'thumb_abduction',
            'threshold': 0.3,
            'low_weight_config': {
                'joints': [2, 3, 4],
                'weights': {'v1': [1, 0, 0], 'v2': [1, 0, 0]},
                'reverse_motion': {'v1': False, 'v2': False}
            },
            'high_weight_config': {
                'joints': [2, 3, 4],
                'weights': {'v1': [0.3, 0.0, 0.7], 'v2': [0.3, 0.0, 0.7]},
                'reverse_motion': {'v1': False, 'v2': False}
            }
        },
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.2,
            'extended_exp_factor': 5
        }
    },
    'thumb_end_flexion': {
        'name': '拇指指尖弯曲',
        'joints': [2, 3, 4],
        'weights': {
            'v1': [0, 0.0, 1],
            'v2': [0, 0.0, 1]
        },
        'robot_idx': 3,
        'type': 'thumb',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 0.9,
            'extended_exp_factor': 1.5
        }
    },
    'index_roll': {
        'name': '食指',
        'joints': [5],
        'weights': {
            'v1': [1],
            'v2': [1]
        },
        'robot_idx': 5,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': False,
            'scale_factor': 1.0,
        }
    },
    'index_root_flexion': {
        'name': '食指',
        'joints': [6, 7, 8],
        'weights': {
            'v1': [1, 0.0, 0],
            'v2': [1, 0.0, 0]
        },
        'robot_idx': 6,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.2,
            'extended_exp_factor': 5
        }
    },
    'index_end_flexion': {
        'name': '食指',
        'joints': [6, 7, 8],
        'weights': {
            'v1': [0, 0.0, 1],
            'v2': [0, 0.0, 1]
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
            'scale_factor': 0.9,
            'extended_exp_factor': 1.5
        }
    },
    'middle_roll': {
        'name': '中指',
        'joints': [9],
        'weights': {
            'v1': [1],
            'v2': [1]
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
            'extended_exp_factor': 5
        }
    },
    'middle_root_flexion': {
        'name': '中指',
        'joints': [10, 11, 12],
        'weights': {
            'v1': [1, 0.0, 0],
            'v2': [1, 0.0, 0]
        },
        'robot_idx': 10,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.2,
            'extended_exp_factor': 5
        }
    },
    'middle_end_flexion': {
        'name': '中指',
        'joints': [10, 11, 12],
        'weights': {
            'v1': [0, 0.0, 1],
            'v2': [0, 0.0, 1]
        },
        'robot_idx': 11,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 0.9,
            'extended_exp_factor': 1.5
        }
    },
    'ring_roll': {
        'name': '无名指',
        'joints': [13],
        'weights': {
            'v1': [1],
            'v2': [1]
        },
        'robot_idx': 13,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': False,
            'scale_factor': 1.0,
        }
    },
    'ring_root_flexion': {
        'name': '无名指',
        'joints': [14, 15, 16],
        'weights': {
            'v1': [1, 0.0, 0],
            'v2': [1, 0.0, 0]
        },
        'robot_idx': 14,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.2,
            'extended_exp_factor': 5
        }
    },
    'ring_end_flexion': {
        'name': '无名指',
        'joints': [14, 15, 16],
        'weights': {
            'v1': [0, 0.0, 1],
            'v2': [0, 0.0, 1]
        },
        'robot_idx': 15,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 0.9,
            'extended_exp_factor': 1.5
        }
    },
    'pinky_roll': {
        'name': '小指',
        'joints': [17],
        'weights': {
            'v1': [1],
            'v2': [1]
        },
        'robot_idx': 17,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': False,
            'scale_factor': 1.0,
        }
    },
    'pinky_root_flexion': {
        'name': '小指',
        'joints': [18, 19, 20],
        'weights': {
            'v1': [1, 0.0, 0],
            'v2': [1, 0.0, 0]
        },
        'robot_idx': 18,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 1.2,
            'extended_exp_factor': 5
        }
    },
    'pinky_end_flexion': {
        'name': '小指',
        'joints': [18, 19, 20],
        'weights': {
            'v1': [0, 0.0, 1],
            'v2': [0, 0.0, 1]
        },
        'robot_idx': 19,
        'type': 'finger',
        'reverse_motion': {
            'v1': False,
            'v2': False
        },
        'dynamic_weight': None,
        'extended_mapping': {
            'enabled': True,
            'scale_factor': 0.9,
            'extended_exp_factor': 1.5
        }
    }
}

MAPPING_ORDER = [
    'thumb_rotate', 'thumb_abduction', 'thumb_root_flexion', 'thumb_end_flexion', 
    'index_roll', 'index_root_flexion', 'index_end_flexion', 
    'middle_roll', 'middle_root_flexion', 'middle_end_flexion', 
    'ring_roll', 'ring_root_flexion','ring_end_flexion', 
    'pinky_roll', 'pinky_root_flexion', 'pinky_end_flexion'
]

MULTI_SEGMENT_CONFIG = {
    'states': [
        'original', 
        'opose', 
        'fist'
        ],
    'state_names': {
        'original': '张手',
        'opose': 'O手势',
        'fist': '握拳'
    }
}
MULTI_SEGMENT_CONFIG_FROZEN = tuple(MULTI_SEGMENT_CONFIG['states'])

ROBOT_ORIGINAL_LEFT = [
    0.0, 0.0, 0.0, 0.0, 0.0, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.15, 0.0, 0.0, 0.0, -0.15, 0.0, 0.0, 0.0
]

ROBOT_ORIGINAL_RIGHT = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0
]

ROBOT_FIST_RIGHT = [
    0.5, 1.57, 0.6, 1.2, 1.2,
    0.18, 1.33, 1.77, 1.43,
    0.18, 1.33, 1.77, 1.43,
    0.18, 1.33, 1.77, 1.43,
    0.18, 1.33, 1.77, 1.43
]

ROBOT_FIST_LEFT = [
    0.5, 1.57, 0.6, 1.2, 1.2,
    0.18, 1.33, 1.77, 1.43,
    0.18, 1.33, 1.77, 1.43,
    0.18, 1.33, 1.77, 1.43,
    0.18, 1.33, 1.77, 1.43
]

ROBOT_OPOSE_LEFT = [
    0.0, 1.2, 0.3, 0.8, 0.0, 0.0, 0.65, 1.0, 0.0, 0.0, 0.65, 1.0, 0.0, 0.0, 0.65, 1.0, 0.0, 0.0, 0.65, 1.0, 0.0
]

ROBOT_OPOSE_RIGHT = [
    0.0, 1.2, 0.3, 0.8, 0.0, 0.0, 0.65, 1.0, 0.0, 0.0, 0.65, 1.0, 0.0, 0.0, 0.65, 1.0, 0.0, 0.0, 0.65, 1.0, 0.0
]

MOTOR_CONSTRAINTS = {
    'left': [
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': True},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': True},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
    ],
    'right': [
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': True},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': True},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
        {'min': 0, 'max': 255, 'enabled': False},
    ]
}