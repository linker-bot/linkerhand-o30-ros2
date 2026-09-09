# hand_config_const.py
from typing import Dict, List, Optional
from dataclasses import dataclass, field
from types import MappingProxyType

@dataclass(frozen=True)        # frozen=True 让实例真正只读
class HandConfig:
    joint_names: List[str] = field(default_factory=list)
    joint_names_en: Optional[List[str]] = None
    min_rad: List[float] = field(default_factory=list)
    max_rad: List[float] = field(default_factory=list)
    init_pos: List[int] = field(default_factory=list)
    preset_actions: Optional[Dict[str, List[int]]] = None

# ------------------------------------------------------------------
# 常量字典（仅构建一次）
# ------------------------------------------------------------------
_HAND_CONFIGS: Dict[str, HandConfig] = {
    "O30": HandConfig(
        joint_names= ['拇指横滚', '拇指航向', '食指航向', '中指航向', '无名指航向', '小指航向', '拇指指根1', '食指指根1', '中指指根1', '无名指指根1', '小指指根1', '食指指根2', '中指指根2', '无名指指根2', '小指指根2', '拇指指尖', '食指指尖', '中指指尖', '无名指指尖', '小指指尖'],
        joint_names_en = ['thumb_cmc_roll', 'thumb_cmc_yaw', 'index_mcp_roll', 'middle_mcp_roll', 'ring_mcp_roll', 'pinky_mcp_roll', 'thumb_mcp', 'index_mcp_pitch', 'middle_mcp_pitch', 'ring_mcp_pitch', 'pinky_mcp_pitch', 'index_pip', 'middle_pip', 'ring_pip', 'pinky_pip', 'thumb_ip', 'index_dip', 'middle_dip', 'ring_dip', 'pinky_dip'],
        min_rad= [0.000, 0.000, -0.400, -0.380, -0.280, -0.280, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000],

        max_rad= [0.611, 2.094, 0.037, 0.054, 0.188, 0.281, 1.713, 1.729, 1.884, 1.963, 1.850, 1.661, 1.707, 1.662, 1.703, 1.733, 1.763, 1.586, 1.675, 1.733],


        init_pos=[172, 0, 169, 169, 162, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        preset_actions={
            "赞": [0, 108, 255, 212, 212, 162, 0, 232, 255, 255, 255, 255, 252, 255, 249, 0, 255, 255, 255, 255],
            "握拳": [0, 198, 255, 212, 212, 162, 50, 232, 255, 255, 255, 255, 252, 255, 249, 255, 255, 255, 255, 255],
            "壹": [170, 255, 255, 202, 162, 71, 39, 0, 255, 255, 255, 0, 252, 255, 249, 154, 0, 255, 255, 255],
            "贰": [242, 255, 69, 219, 162, 71, 67, 0, 0, 255, 255, 0, 0, 255, 249, 154, 0, 0, 255, 255],
            "叁": [242, 255, 69, 158, 207, 71, 56, 0, 0, 0, 255, 0, 0, 0, 249, 154, 0, 0, 0, 255],
            "肆": [59, 231, 169, 169, 162, 170, 151, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 0, 0],
            "伍": [172, 0, 169, 169, 162, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "陆": [75, 0, 255, 202, 162, 255, 0, 255, 255, 255, 0, 255, 252, 255, 0, 0, 255, 255, 255, 0],
            "柒": [234, 132, 255, 202, 162, 71, 54, 159, 171, 255, 255, 93, 90, 255, 249, 89, 120, 121, 255, 255],
            "捌": [56, 0, 255, 202, 162, 71, 0, 0, 255, 255, 255, 0, 252, 255, 249, 0, 0, 255, 255, 255],
            "1-1": [55, 181, 225, 176, 212, 162, 122, 164, 0, 0, 0, 139, 0, 0, 0, 0, 0, 0, 0, 0],
            "张开": [172, 0, 169, 169, 162, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

        }
    ),
}
HAND_CONFIGS = MappingProxyType(_HAND_CONFIGS)
