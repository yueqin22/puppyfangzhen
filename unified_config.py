"""统一配置访问模块 (P1-6)

提供 C++ 和 Python 共享的单一参数源访问接口。
优先级: 环境变量 > YAML 配置文件 > 代码默认值

用法:
    from unified_config import get_param, get_section

    n_particles = get_param('amcl', 'n_particles', 300)
    use_amcl = get_param('navigation', 'use_amcl', True)
    amcl_cfg = get_section('amcl')
"""
import os
import yaml

_CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config')
_UNIFIED_FILE = os.path.join(_CONFIG_DIR, 'unified_params.yaml')
_cache = None

# 环境变量到配置键的映射
_ENV_MAP = {
    'USE_AMCL': ('navigation', 'use_amcl', lambda v: v == '1'),
    'USE_RVO': ('navigation', 'use_rvo', lambda v: v == '1'),
    'USE_TEB': ('navigation', 'use_teb', lambda v: v == '1'),
    'USE_EVAL': ('navigation', 'use_eval', lambda v: v == '1'),
    'USE_IMU_FUSION': ('navigation', 'use_imu_fusion', lambda v: v == '1'),
    'SEED': ('sim', 'seed', int),
    'PROFILE': (None, 'profile', str),
    'SIM_DT': ('sim', 'dt', float),
    'MAX_FRAMES': ('sim', 'max_frames', int),
    'ROBOT_RADIUS': ('robot', 'radius', float),
    'MAX_LINEAR_X': ('robot', 'max_linear_x', float),
    'MAX_ANGULAR_Z': ('robot', 'max_angular_z', float),
    'AMCL_PARTICLES': ('amcl', 'n_particles', int),
    'AMCL_SIGMA_OBS': ('amcl', 'sigma_obs', float),
    'AMCL_KLD_MIN': ('amcl', 'kld_min', int),
    'AMCL_KLD_MAX': ('amcl', 'kld_max', int),
    'COSTMAP_INFLATION': ('costmap', 'inflation_radius', float),
    'CMD_TIMEOUT': ('robot', 'cmd_timeout', int),
    'LOW_BATTERY': ('safety', 'low_battery', int),
    'CRITICAL_BATTERY': ('safety', 'critical_battery', int),
    'MOTORS_ENABLED_ON_START': ('safety', 'motors_enabled_on_start',
                                lambda v: v == '1'),
}


def _load_unified(force_reload=False):
    """加载统一配置文件。"""
    global _cache
    if _cache is not None and not force_reload:
        return _cache
    if not os.path.isfile(_UNIFIED_FILE):
        _cache = {}
        return _cache
    try:
        with open(_UNIFIED_FILE, 'r', encoding='utf-8') as f:
            _cache = yaml.safe_load(f) or {}
    except Exception as e:
        print(f"[WARN] 加载统一配置失败: {e}")
        _cache = {}
    return _cache


def get_param(section, key, default=None):
    """获取参数值，环境变量优先。

    Args:
        section: 配置段名 (如 'amcl', 'sim', 'robot')
        key: 参数键名
        default: 默认值

    Returns:
        参数值 (环境变量 > YAML > default)
    """
    # 1. 检查环境变量覆盖
    for env_name, (s, k, conv) in _ENV_MAP.items():
        if s == section and k == key:
            env_val = os.environ.get(env_name)
            if env_val is not None:
                try:
                    return conv(env_val)
                except (ValueError, TypeError):
                    pass

    # 2. 从 YAML 读取
    cfg = _load_unified()
    if section is None:
        return cfg.get(key, default)
    return cfg.get(section, {}).get(key, default)


def get_section(section):
    """获取整个配置段。"""
    cfg = _load_unified()
    return cfg.get(section, {})


def get_profile():
    """获取当前 profile 名称。"""
    return get_param(None, 'profile', 'dev')


def reload():
    """强制重新加载配置。"""
    _load_unified(force_reload=True)


def export_for_cpp(output_path=None):
    """导出为 C++ mini_yaml 可解析的格式。

    C++ 端的 sim/mini_yaml.h 解析标准 YAML，因此直接复制即可。
    此方法主要用于验证配置一致性。
    """
    if output_path is None:
        output_path = os.path.join(_CONFIG_DIR, 'unified_params_export.yaml')
    cfg = _load_unified()
    with open(output_path, 'w', encoding='utf-8') as f:
        yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True)
    return output_path


def validate():
    """验证配置完整性，返回 (is_valid, errors)。"""
    cfg = _load_unified()
    errors = []

    required_sections = ['sim', 'navigation', 'robot', 'amcl',
                         'costmap', 'planner', 'obstacles', 'safety']
    for sec in required_sections:
        if sec not in cfg:
            errors.append(f"缺少配置段: {sec}")

    # AMCL 参数范围检查
    amcl = cfg.get('amcl', {})
    if amcl.get('kld_min', 50) > amcl.get('kld_max', 500):
        errors.append("amcl.kld_min > amcl.kld_max")
    if amcl.get('n_particles', 300) < 1:
        errors.append("amcl.n_particles < 1")

    # 仿真参数检查
    sim = cfg.get('sim', {})
    if sim.get('dt', 0.0333) <= 0:
        errors.append("sim.dt <= 0")
    if sim.get('max_frames', 108000) < 1:
        errors.append("sim.max_frames < 1")

    # 安全参数检查
    safety = cfg.get('safety', {})
    if safety.get('low_battery', 20) <= safety.get('critical_battery', 10):
        errors.append("safety.low_battery <= safety.critical_battery")

    return len(errors) == 0, errors


if __name__ == '__main__':
    import json
    valid, errors = validate()
    print(f"配置验证: {'通过' if valid else '失败'}")
    if errors:
        for e in errors:
            print(f"  - {e}")
    print(f"\n当前 profile: {get_profile()}")
    print(f"AMCL 粒子数: {get_param('amcl', 'n_particles', 300)}")
    print(f"仿真步长: {get_param('sim', 'dt', 0.0333)}")
    print(f"USE_AMCL={os.environ.get('USE_AMCL', '未设置')} → "
          f"use_amcl={get_param('navigation', 'use_amcl', True)}")
