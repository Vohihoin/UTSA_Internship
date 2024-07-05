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