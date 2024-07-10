import Scoring

class Person:
    def __init__(self, neck, trunk, legs, load, upperArm, lowerArm, wrist, couplingScore, activityScore):
        
        self.neck = neck
        self.trunk = trunk
        self.legs = legs
        self.load = load
        self.upperArm = upperArm
        self.lowerArm = lowerArm
        self.wrist = wrist
        self.couplingScore = couplingScore
        self.activityScore = activityScore

    def getREBAScore(self):
        # This is gotten using the REBA Algorithm
        postureAScore = Scoring.getPostureScoreA(self.neck.getScore(), self.trunk.getScore(), self.legs.getScore())
        postureBScore = Scoring.getPostureScoreB(self.lowerArm.getScore(), self.upperArm.getScore(), self.wrist.getScore())

        scoreA = postureAScore + self.load.getScore()
        scoreB = postureBScore + self.couplingScore
        scoreC = Scoring.getScoreC(scoreA, scoreB)
        return scoreC + self.activityScore
    
    def getPostureScoreA(self):
        return Scoring.getPostureScoreA(self.neck.getScore(), self.trunk.getScore(), self.legs.getScore())

    def getPostureScoreB(self):
        return Scoring.getPostureScoreB(self.lowerArm.getScore(), self.upperArm.getScore(), self.wrist.getScore())
    
    def getScoreA(self):

        postureAScore = Scoring.getPostureScoreA(self.neck.getScore(), self.trunk.getScore(), self.legs.getScore())
        return postureAScore + self.load.getScore()
    
    def getScoreB(self):

        postureBScore = Scoring.getPostureScoreB(self.lowerArm.getScore(), self.upperArm.getScore(), self.wrist.getScore())
        return postureBScore + self.couplingScore
    
    def getScoreC(self):

        postureAScore = Scoring.getPostureScoreA(self.neck.getScore(), self.trunk.getScore(), self.legs.getScore())
        postureBScore = Scoring.getPostureScoreB(self.lowerArm.getScore(), self.upperArm.getScore(), self.wrist.getScore())

        scoreA = postureAScore + self.load.getScore()
        scoreB = postureBScore + self.couplingScore
        return Scoring.getScoreC(scoreA, scoreB)
    
    def scoreTable(self):

        returnString = "PostureScoreA: " + str(self.getPostureScoreA()) + "\nPostureScoreB: " + str(self.getPostureScoreB()) + "\nScoreA: " + str(self.getScoreA()) +  "\nScoreB: " + str(self.getScoreB())  + "\nScoreC: " + str(self.getScoreC()) + "\nREBA Score: " + str(self.getREBAScore()) 
        return returnString