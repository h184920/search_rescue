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

#slår sammen "celle_størrelse x celle_størrelse" celler for å skalere ned. Dette da på w1 (bare den som er sjekket) er 2000x2000 som er alt for mye og bare lager mer styr
def ned_skaler_grid(grid, celle_størrelse):
    h = grid.shape[0] // celle_størrelse**2
    b = grid.shape[1] //celle_størrelse**2
    cellen = grid[:h, :b].reshape((h // celle_størrelse), celle_størrelse, (b // celle_størrelse), celle_størrelse) #veldig støgt ik men er bare cellen (i,j) i den nye ruten (R,C)
    Ikkr_vegger = np.where((cellen == UKJENT).any(axis=(1,3)), UKJENT, LEDIG)
    return np.where((cellen == VEGG).any(axis =(1,3)), VEGG, Ikkr_vegger).astype(np.uint8)

