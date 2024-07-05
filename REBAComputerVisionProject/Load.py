class Load:

    def __init__(self, amount, shock):
        self.amount = amount
        self.shock = shock

    def setAmount(self, amount):
        self.amount = amount

    def getAmount(self):
        return self.amount
    
    def setShock(self, shock):
        self.shock = shock
    
    def getShock(self):
        return self.shock
    
    def getScore(self):

        score = 0

        if self.amount >= 0 and self.amount < 11:
            score += 0
        elif self.amount >= 11 and self.amount < 22:
            score += 1
        elif self.amount >= 22:
            score += 2
        
        if self.shock:
            score += 1

        return score