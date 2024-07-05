import math

"""
Gets vector from points
"""
def getVector(point1, point2):
    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]
    dz = point1[2] - point2[2]
    return((dx, dy, dz))

def dotProduct(vector1, vector2):

    return ((vector1[0]*vector2[0]) + (vector1[1]*vector2[1]) + (vector1[2]*vector2[2]))


def magnitude(vector):
    magnitude = math.sqrt( vector[0]**2 + vector[1]**2 + vector[2]**2 )
    return magnitude

def getAngleBetweenVectors(a, b):

    angle = math.degrees( math.acos(dotProduct(a, b) / (magnitude(a) * magnitude(b)) ) )
    return(angle)



