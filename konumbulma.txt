from math import radians, sin, cos, sqrt
def get_distance_metres(aLocation1, aLocation2):
    dlat = radians(aLocation2.lat - aLocation1.lat)
    dlong = radians(aLocation2.lon - aLocation1.lon)
    a = sin(dlat / 2) * sin(dlat / 2) + cos(radians(aLocation1.lat)) \
        * cos(radians(aLocation2.lat)) * sin(dlong / 2) * sin(dlong / 2)
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    d = 6371000 * c
    return d
