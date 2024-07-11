import math
import decimal

"""
Gets vector from points
"""
def getVector(tail,head):
    dx = head[0] - tail[0]
    dy = head[1] - tail[1]
    dz = head[2] - tail[2]
    return((dx, dy, dz))

def dotProduct(vector1, vector2):

    return ((vector1[0]*vector2[0]) + (vector1[1]*vector2[1]) + (vector1[2]*vector2[2]))


def magnitude(vector):
    magnitude = math.sqrt( vector[0]**2 + vector[1]**2 + vector[2]**2 )
    return magnitude

def getAngleBetweenVectors(a, b):

    angle = math.degrees( math.acos(dotProduct(a, b) / (magnitude(a) * magnitude(b)) ) )
    return(angle)

def vectorMultiplicationByScalar(vector, scalar):

    return (vector[0]*scalar, vector[1]*scalar, vector[2]*scalar)

def vectorDivisionByScalar(vector, scalar):

    return (vector[0]/scalar, vector[1]/scalar, vector[2]/scalar)

def getUnitVector(vector):

    mag = magnitude(vector)

    return vectorDivisionByScalar(vector, mag)

def addVectors(vector1, vector2):

    return (vector1[0]+vector2[0], vector1[1]+vector2[1], vector1[2]+vector2[2])

def subtractVectors(vector1, vector2):

    return (vector1[0]-vector2[0], vector1[1]-vector2[1], vector1[2]-vector2[2])

def midPoint(point1, point2):

    return vectorDivisionByScalar( (point1[0]+point2[0], point1[1]+point2[1], point1[2]+point2[2]), 2 )

def roundVectorValues(vector):

    return ( round(vector[0], 3), round(vector[1], 3), round(vector[2], 3) )
    
def getDistanceBetweenPoints(point1, point2):

    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]
    dz = point1[2] - point2[2]

    return math.sqrt(dx**2 + dy**2 + dz**2)



