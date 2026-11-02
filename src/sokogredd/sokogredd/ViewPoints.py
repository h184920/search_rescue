import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from visualization_msgs.msg import Marker

import cv2 
import numpy as np 

LEDIG = 0
VEGG = 1
SETT = 2
UKJENT = -1

def disk(radius_celler): #insanly kul måte å få en sirkelformet maske for scan radiusen vi trenger
    k = 2*int(np.ceil(radius_celler)) + 1 #finner maske størrelsen 
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k,k)) #git ut en kxk numpy matrise med denne masken i midetn

#utvider masken for "naboene". dilate trenger tall så må omgjøre på veien
def utvid_for_disk(masken, mal):
    return cv2.dilate(masken.astype(np.uint8), mal).astype(bool)

