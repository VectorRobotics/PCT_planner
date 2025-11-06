class ConfigPlanner():
    use_quintic = True
    max_heading_rate = 10
    astar_cost_threshold = 25.0  # Maximum traversability cost for goal (default was 20)
    safe_cost_margin = 15.0  # Safety margin for cost calculations


class ConfigWrapper():
    tomo_dir = '/rsc/tomogram/'


class Config():
    planner = ConfigPlanner()
    wrapper = ConfigWrapper()