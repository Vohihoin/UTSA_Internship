class LowerArm:

    def __init__(self, angle):
        self.angle = angle

    def setAngle(self, angle):
        self.angle = angle

    def getAngle(self):
        return self.angle
    
    def getScore(self):
        score = 0

        if self.angle > 60 and self.angle < 100:
            score += 1
        elif self.angle >= 0 and self.angle <= 60:
            score += 2
        elif self.angle >= 100:
            score += 3
        
        return score