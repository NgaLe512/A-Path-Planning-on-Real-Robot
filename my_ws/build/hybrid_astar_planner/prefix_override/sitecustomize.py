import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nga/robot_social_pathfinding/my_ws/install/hybrid_astar_planner'
