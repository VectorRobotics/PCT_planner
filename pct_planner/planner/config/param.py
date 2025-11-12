class ConfigPlanner():
    use_quintic = True
    max_heading_rate = 10
    astar_cost_threshold = 25.0  # Maximum traversability cost for goal (default was 20)
    safe_cost_margin = 15.0  # Safety margin for cost calculations

    # Smoothing parameters (GPMP Optimizer)
    sample_interval = 3
    interpolate_num = 8
    max_iterations = 100
    lambda_initial = 10.0  # Optimal value: 5.0 too aggressive, 200.0 too conservative
    qc_position = 0.05  # Reduced from 0.1 for smoother position transitions
    qc_heading = 0.005  # Reduced from 0.01 for smoother heading changes

    # Debug flags
    enable_path_debug = True  # Default to True to help with debugging
    enable_optimizer_debug = True


class ConfigWrapper():
    tomo_dir = '/rsc/tomogram/'


class Config():
    planner = ConfigPlanner()
    wrapper = ConfigWrapper()