import numpy as np
from pyproj import Proj

def longlat2utm(long, lat, hemisphere='N'):
    '''
    Converts latitude and longitude to UTM coordinates
    Using the pyproj library
    # Using the simplified formula from Krüger 1912, explained [here][https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system]

    # @book{kruger1912konforme,
    # title={Konforme Abbildung des Erdellipsoids in der Ebene},
    # author={Kr{\"u}ger, Louis},
    # number={52},
    # year={1912},
    # publisher={BG Teubner}
    # }

    In:
        long: longitude in degrees
        lat: latitude in degrees
        hemisphere: 'N' or 'S' for northern or southern hemisphere
    
    Out:
        N: northing in meters
        E: easting in meters
    '''
    if type(long) == list:
        zone = _getUTMZone(long[0])
    else:
        zone = _getUTMZone(long)
    hem = "north" if hemisphere == 'N' else "south"
    myproj = Proj(f"+proj=utm +zone={zone} +{hem} +datum=WGS84 +units=m +no_defs ")
    N, E = myproj(long, lat)
    return N, E

def _getReferenceMeridian(long):
    '''
    Returns the longitude of the central meridian of the UTM zone

    Out:
        ref_meridian: (float) longitude of the central meridian in degrees
    '''
    zone = _getUTMZone(long)
    return (zone - 1)*6 - 180 + 3

def _getUTMZone(long):
    '''
    Returns the UTM zone for a given longitude

    Out: 
        zone: (int) UTM zone
    '''
    return round((long + 180)/6)