def getVector(tail, head):

    return (head[0] - tail[0], head[1] - tail[1], head[2] - tail[2])

def vectorToXMLString(vector):

    return str(vector[0]) + " " + str(vector[1]) + " "  + str(vector[2])