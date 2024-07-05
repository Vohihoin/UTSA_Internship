def getScoreFromPosition(angle, twisted, sideBending):
    score = 0
    if angle >= 0 and angle < 20:
        score = 1
    elif angle >= 20:
        score = 2
    elif angle < 0:
        score = 2

    if twisted:
        score += 1
    if sideBending:
        score +=1

    return score


class Neck:
    
    def __init__(self, angle, twisted, sideBending) -> None:

        self.angle = angle
        self.twisted = twisted
        self.sideBending = sideBending

    def setAngle(self, angle):
        self.angle = angle

    def getAngle(self):
        return self.angle
    
    def setTwisted(self, twisted):
        self.twisted = twisted

    def getTwisted(self):
        return self.twisted
    
    def setSideBending(self, sideBending):
        self.sideBending = sideBending
    
    def getSideBending(self):
        return self.sideBending
    
    def getScore(self):
        return getScoreFromPosition(self.angle, self.twisted, self.sideBending)

