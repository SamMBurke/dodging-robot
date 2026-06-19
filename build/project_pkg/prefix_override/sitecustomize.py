import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sam/Documents/VSCode/ME 640/Project/project_ws/install/project_pkg'
