import random


#MODULE FOR KEEPING TRACK OF RUNNING MIN, MAX, AND AVG
#PACHECK, NICHOLAS
#APART OF GORA SOFTWARE PACKAGE

class Running_Stats:
    def __init__(self):
        self.prev_min, self.prev_max = 1000000, -1000000

    def running_stats(self, value):

        if value > self.prev_max:
            self.prev_max = value
        elif value < self.prev_min:
            self.prev_min = value

        return self.prev_min, self.prev_max

if __name__ == "__main__":
    RS = Running_Stats()
    while True:

        value = random.randint(-10000, 10000)

        mined, maxed = RS.running_stats(value)
        print(value,mined, maxed)