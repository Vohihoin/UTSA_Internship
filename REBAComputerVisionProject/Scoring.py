# 3 x 5 x 4 matrix for scoring


postureScoreARubric = [
    [[1,2,3,4],
     [2,3,4,5],
     [2,4,5,6],
     [3,5,6,7],
     [4,6,7,8]], #Neck1

    [[1,2,3,4],
     [3,4,5,6],
     [4,5,6,7],
     [5,6,7,8],
     [6,7,8,9]], #Neck2

    [[3,3,5,6],
     [4,5,6,7],
     [5,6,7,8],
     [6,7,8,9],
     [7,8,9,9]]  #Neck3
]

#2 x 6 x 3 matrix for B
postureScoreBRubric = [

    [[1,2,2],
     [1,2,3],
     [3,4,5],
     [4,5,5],
     [6,7,8],
     [7,8,8]
     ], #Lower Arm 1
    [[1,2,3],
     [2,3,4],
     [4,5,5],
     [5,6,7],
     [7,8,8],
     [8,9,9]
     ], # Lower Arm 2

]

scoreCRubric = [

    [1,1,1,2,3,3,4,5,6,7,7,7],
    [1,2,2,3,4,4,5,6,6,7,7,8],
    [2,3,3,3,4,5,6,7,7,8,8,8],
    [3,4,4,4,5,6,7,8,8,9,9,9],
    [4,4,4,5,6,7,8,8,9,9,9,9],
    [6,6,6,7,8,8,9,9,10,10,10,10],
    [7,7,7,8,9,9,9,10,10,11,11,11],
    [8,8,8,9,10,10,10,10,10,11,11,11],
    [9,9,9,10,10,10,11,11,11,12,12,12],
    [10,10,10,11,11,11,11,12,12,12,12,12],
    [11,11,11,11,12,12,12,12,12,12,12,12],
    [12,12,12,12,12,12,12,12,12,12,12,12]

]

def getPostureScoreA(neck, trunk, legs):

    #Normalizing all the scores
    if neck > 3:
        neck = 3
    if trunk > 5:
        trunk = 5
    if legs > 4:
        legs = 4

    # Lowering the indexes to account for indexes starting at 0
    neck -= 1
    trunk -= 1
    legs -= 1


    return postureScoreARubric[neck][trunk][legs]

def getScoreA(postureScoreA, loadScore):
    return postureScoreA + loadScore

def getPostureScoreB(lowerArm, upperArm, wrist):

    # Normalizing values
    if lowerArm > 2:
        lowerArm = 2
    if upperArm > 6:
        upperArm = 6
    if wrist > 3:
        wrist = 3

    #Adjusting for index starting at 0
    
    lowerArm -= 1
    upperArm -= 1
    wrist -= 1

    return postureScoreBRubric[lowerArm][upperArm][wrist]

def getScoreB(postureScoreB, couplingScore):
    return postureScoreB + couplingScore

def getScoreC(scoreA, scoreB):
    return scoreCRubric[scoreA][scoreB]

def getREBAScore(scoreC, activityScore):
    return scoreC + activityScore