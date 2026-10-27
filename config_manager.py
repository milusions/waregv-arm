"""YAML configuration loading, defaults, and persistence."""
import os
import yaml


DEFAULT_CONFIG = {
    'arm_parameters': {'L1': 100.0, 'L2': 80.0, 'L3': 60.0},
    'constraints': {'min_x': 0.0, 'min_y': -50.0},
    'animation': {'duration_s': 1.0, 'bezier_weight': 3.0},
    'initial_target': {'x': 120.0, 'y': 100.0},
    'hardware': {'enabled': True, 'fps': 30},
    'waypoint_dwell_s': 0.0,
    'waypoints': [],
        "matlab_script_dir": "matlab",
"algorithm_source": "INBUILT"
}


def load_config(config_file="config.yaml"):
    """Load config from disk, creating a default file if missing."""
    if not os.path.exists(config_file):
        with open(config_file, 'w') as f:
            yaml.dump(DEFAULT_CONFIG, f, default_flow_style=False)

    with open(config_file, 'r') as f:
        return yaml.safe_load(f)