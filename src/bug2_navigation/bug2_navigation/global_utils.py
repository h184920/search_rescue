from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

from math import pi, acos, sqrt


def extract_odom_position(msg: Odometry) -> tuple:
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    return x, y

def extract_odom_orientation(msg: Odometry) -> float:
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    orientation = euler[2]
    return orientation

def compute_distance_and_offset(x, y, orient, targ_x, targ_y):
    dx, dy = targ_x - x, targ_y - y
    dist = sqrt(dx**2 + dy**2)
    if dist == 0.0:
        return dist, orient
    off = orient - (1 if dy >= 0.0 else -1)*acos(dx/dist)
    if off < -pi or pi < off:
        off = (off % pi) * off/abs(off) * (-1)
    return dist, off

def logg(loggerName, *args):
    if len(args) == 1:
        args = args[0]
    s = '%s:\n%s\n'%(loggerName, str(args))
    return s.replace('\n', '\n'+' '*10)

def prefix(inst, name):
    return '/'+inst.namespace+'/'+name
