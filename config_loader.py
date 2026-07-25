"""Configuration loader for YAML-based parameter management.
Reads config/*.yaml files and exposes parameters as a nested dict."""
import os
import yaml

CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config')
_cache = None

def load_config(force_reload=False):
    """Load all YAML config files from config/ directory.
    Returns a dict with keys matching filenames (without .yaml)."""
    global _cache
    if _cache is not None and not force_reload:
        return _cache
    config = {}
    if not os.path.isdir(CONFIG_DIR):
        return config
    for fname in os.listdir(CONFIG_DIR):
        if fname.endswith('.yaml') or fname.endswith('.yml'):
            key = fname.rsplit('.', 1)[0]
            path = os.path.join(CONFIG_DIR, fname)
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    config[key] = yaml.safe_load(f) or {}
            except Exception as e:
                print(f"[WARN] Failed to load config {fname}: {e}")
                config[key] = {}
    _cache = config
    return config

def get(section, key, default=None):
    """Get a parameter value: get('planner_local', 'max_v', 0.3)."""
    cfg = load_config()
    return cfg.get(section, {}).get(key, default)

if __name__ == '__main__':
    import json
    print(json.dumps(load_config(), indent=2, default=str))
