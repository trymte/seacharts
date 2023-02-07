import numpy as np
import datetime
import copy
from simcharts_interfaces.msg import Vessel, Polygon, Point

def dictToList(dict):
    '''
    Converts a dictionary to a list of Vessels messages

    In:
        dict: (Dict) dictionary of Vessel messages
    Out:
        list: (Vessel[]) list of Vessel messages
    '''
    vessel_list = []
    for vessel in dict.values():
        vessel_list.append(copy.deepcopy(vessel))
    return vessel_list

def getTimeStamp(clock=None):
    '''
    Returns the current time stamp

    Out:
        timestamp: (Float) current time stamp
    '''
    if clock is None:
        ts = datetime.datetime.now().timestamp()
    else:
        ts = float(f"{clock.now().to_msg().sec}.{clock.now().to_msg().nanosec}")
    return ts

def in_horizon(vessel: Vessel, size, origin) -> bool:
    '''
    Filters out all ships that are not in the vicinity of the origin
    '''
    inHorizon =  (vessel.x < origin[0] + size[0] and vessel.x > origin[0] \
        and vessel.y < origin[1] + size[1] and vessel.y > origin[1])
    return inHorizon

def pointlist_to_polygon(pointlist):
    '''
    Converts a list of points to a polygon

    In:
        pointlist: (List[[x,y], [x,y] ,...]) list of points
    Out:
        polygon: (Polygon) polygon
    '''
    polygon = Polygon()
    polygon.points = []
    for p in pointlist:
        point = Point()
        point.x = p[0]
        point.y = p[1]
        polygon.points.append(point)
    return polygon


def ssa(a, unit='deg'):
    '''
    Returns smallest signed angle
    '''
    if unit == 'rad':
        mod = np.abs(a + np.pi) % (2*np.pi) - np.pi
        a = np.sign(a)*mod
        return a
    if unit == 'deg':
        mod = np.abs(a + 180) % (360) - 180
        a = np.sign(a)*mod
        return a