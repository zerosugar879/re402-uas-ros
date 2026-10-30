import os
from typing import Any, Dict, Optional

import yaml


# Find yaml
def find_params_yaml(package_name: str = "robot_hardware") -> Optional[str]:
    search = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        p = os.path.join(search, 'config', 'params.yaml')
        if os.path.exists(p):
            return p
        parent = os.path.dirname(search)
        if parent == search:
            break
        search = parent

    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(package_name)
        p = os.path.join(share, "config", "params.yaml")
        if os.path.exists(p):
            return p
    except Exception:
        pass

    return None


# Load params
def load_node_params(node, package_name: str = "robot_hardware") -> Optional[Dict[str, Any]]:
    path = find_params_yaml(package_name)
    if path is None:
        node.get_logger().warn("params.yaml not found")
        return None
    with open(path) as f:
        cfg = yaml.safe_load(f)
    node_name = node.get_name()
    if node_name not in cfg:
        node.get_logger().warn(f"Section '{node_name}' not found in params.yaml")
        return None
    return cfg[node_name].get("ros__parameters", {})