import os

# CARLA settings
CARLA_HOST = os.getenv('CARLA_HOST', 'localhost')
CARLA_PORT = 2000
CARLA_TIMEOUT = 120.0

# MAP settings
MAP_NAME = 'Town10HD_Opt'
MAP_X_MIN = -115
MAP_X_MAX =  110
MAP_Y_MIN = -70
MAP_Y_MAX =  145
MAP_IMAGE_NAME = 'town10_map.png'

# Example settings for another map
# MAP_NAME = 'Town07'
# MAP_X_MIN = -203
# MAP_X_MAX =  86
# MAP_Y_MIN = -246 
# MAP_Y_MAX =  125
# MAP_IMAGE_NAME = 'town07_map.png'

# ns-3 settings
NS3_HOST = 'localhost'
NS3_SEND_PORT = 5556
NS3_RECV_PORT = 5557