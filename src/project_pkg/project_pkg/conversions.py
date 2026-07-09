import numpy as np
from geometry_msgs.msg import Point


def corners_to_ros(corners):
    '''
    Converts an (N, 2) numpy array of corner points into a list of geometry_msgs/Point
    '''
    return [Point(x=float(c[0]), y=float(c[1]), z=0.0) for c in corners]

def corners_from_ros(corners_msg):
    '''
    Converts a list of geometry_msgs/Point into an (N, 2) numpy array
    '''
    return np.array([[p.x, p.y] for p in corners_msg])