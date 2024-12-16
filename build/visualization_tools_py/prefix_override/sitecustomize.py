import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/suxin/English_Path/test111/autonomous_navigation_environment_ego/install/visualization_tools_py'
