class Wrist:
    def __init__(self, angle, twisted):
        self.angle = angle
        self.twisted = twisted

    def setAngle(self, angle):
        self.angle = angle

    def getAngle(self):
        return self.angle
    
    def setTwisted(self, twisted):
        self.twisted = twisted

    def getTwisted(self):
        return self.twisted
    
    
    def getScore(self):
        score = 0

        if self.angle < 15 and self.angle > -15:
            score += 1
        elif self.angle >= 15:
            score += 2
        elif self.angle <= -15:
            score += 2
        
        if self.twisted:
            score += 1

        return score