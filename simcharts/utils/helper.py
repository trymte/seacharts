import datetime
import copy
from simcharts_interfaces.msg import Vessel

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

def isInHorizon(vessel: Vessel, size, origin) -> bool:
    '''
    Filters out all ships that are not in the vicinity of the origin
    '''
    inHorizon =  (vessel.x < origin[0] + size[0] and vessel.x > origin[0] \
        and vessel.y < origin[1] + size[1] and vessel.y > origin[1])
    return inHorizon