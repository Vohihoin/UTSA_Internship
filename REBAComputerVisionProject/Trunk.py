class Trunk:
    def __init__(self, angle, twisted, sideBending):
        self.angle = angle
        self.twisted = twisted
        self.sideBending = sideBending

    def setAngle(self,angle):
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
        score = 0

        if self.angle == 0:
            score += 1
        elif self.angle < 0 and self.angle > -20:
            score+= 2
        elif self.angle > 0 and self.angle < 20: 
            score += 2
        elif self.angle >= 20 and self.angle < 60:
            score += 3
        elif self.angle <= -20:
            score += 3
        elif self.angle >= 60:
            score += 4

        if self.twisted:
            score += 1
        if self.sideBending:
            score += 1

        return score