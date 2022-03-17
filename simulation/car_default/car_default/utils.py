from custom_messages.msg import Trajectory
from time import sleep
from numpy.random import randn
from random import random

def ignore_trajectory(obj, params=[]):
    if params:
        obj.get_logger().warn("ignore_trajectory doesn't use any params.")
    obj.trajectory = Trajectory() 

def delayed_reaction(obj, params=[]):
    if params:
        sleep(params[0]*0.001) 
    else:
        obj.get_logger().warn("delayed_reaction requires the time delay as a parameter (array with single element).")

def randomize_trajectory(obj, params=[]):
    if params:
        obj.get_logger().warn("randomize_trajectory doesn't use any params.")

    for idx,point in enumerate(obj.trajectory.points):
        obj.trajectory.points[idx].x = randn()
        obj.trajectory.points[idx].y = randn()

def randomize_speed(obj, params=[]):
    if params:
        obj.speed = random()*params[0]
    else:
        obj.get_logger().warn("randomize_speed takes a scale parameter.")
