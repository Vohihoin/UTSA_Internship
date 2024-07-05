class UpperArm:
    
    def __init__(self, angle, shoulderRaised, armAbducted, armSupported, leaning):

        self.angle = angle
        self.shoulderRaised = shoulderRaised
        self.armAbducted = armAbducted
        self.armSupported = armSupported
        self.leaning = leaning

    def setAngle(self, angle):
        self.angle = angle

    def getAngle(self):
        return self.angle
    
    def setShoulderRaised(self, shoulderRaised):
        self.shoulderRaised = shoulderRaised

    def getShoulderRaised(self):
        return self.shoulderRaised
    
    def setArmAbducted(self, armAbducted):
        self.armAbducted = armAbducted

    def getArmAbducted(self):
        return self.armAbducted   
    
    def setArmSupported(self, armSupported):
        self.angle = armSupported

    def getArmSupported(self):
        return self.armSupported
    
    def setLeaning(self, leaning):
        self.leaning = leaning

    def getLeaning(self):
        return self.leaning  

    def getScore(self):
        score = 0
        
        if self.angle < 20 and self.angle > -20:
            score += 1
        elif self.angle <= -20:
            score += 2
        elif self.angle >= 20 and self.angle < 45:
            score += 2
        elif self.angle >= 45 and self.angle < 90:
            score += 3
        elif self.angle >= 90:
            score += 4

        if self.shoulderRaised:
            score += 1
        if self.armAbducted:
            score += 1
        if self.armSupported or self.leaning:
            score -= 1

        return score
        