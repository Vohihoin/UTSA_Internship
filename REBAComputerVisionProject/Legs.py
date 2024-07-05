class Legs:
    
    def __init__(self, legsDown, angle):
        self.legsDown = legsDown
        self.angle = angle

    def setAngle(self, angle):
        self.angle = angle

    def getAngle(self):
        return self.angle
    
    def setLegsDown(self, legsDown):
        self.legsDown = legsDown

    def getLegsDown(self):
        return self.legsDown
    
    def getScore(self):
        score = 0
        if self.legsDown:
            score += 1
        else:
            score += 2
        
        if self.angle > 30 and self.angle < 60:
            score += 1
        elif self.angle >= 60:
            score += 2

        return score
