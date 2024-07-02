import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/neslisah/Documents/2023-2024Spring/ME462/MiniROS/dew_ws/install/mini_control'
