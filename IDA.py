# 8-puzzle-IDA-
import copy
import math
from time import clock
import random
import numpy
# from DFS3 import *

global GOAL,GOAL_POS,THRESHOLD,NEXT_THRS,PATH,count_ida
GOAL = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]
THRESHOLD=0
NEXT_THRS=float('Inf')
count_ida=0



def solvable(puzzle):
    a = puzzle.list
    count = 0
    for x in range(len(a)):
        for y in range(x + 1, len(a)):
            if a[y] != 0 and a[x] > a[y]:
                count = count + 1
    if count == 0:
        return True
    if (count % 2) == 0:
        return True
    else:
        return False


def get_pos(grid):
    pos = numpy.zeros((9,2),int)
    for i in range(3):
        for j in range(3):
            num=grid[i][j]
            pos[num][0] = i
            pos[num][1]=j
    return pos

class State:
    def __init__(self, grid, parent_state, direction):
        self.grid = grid.copy()
        self.pos=get_pos(self.grid)
        self.list=numpy.reshape(self.grid, 9).tolist()
        if parent_state is None:
            self.p = None
            self.direction = None
        else:
            self.p = parent_state
            self.direction = direction
        self.h = self.__manhattan__()
        self.g = self.__movement_num__()
        self.f = self.g + self.h
        self.sovable=self.__solvable__()

    def __manhattan__(self):
        distance = 0
        for num in range(1,9):
            i,j=self.pos[num][0],self.pos[num][1]
            x,y=GOAL_POS[num][0],GOAL_POS[num][1]
            distance = distance + int(math.fabs(i - x) + math.fabs(j - y))
        return distance

    def __movement_num__(self):
        if self.p is not None:
            return self.p.g + 1
        else:
            return 0


    def __solvable__(self):
        a = self.list
        count = 0
        for x in range(len(a)):
            for y in range(x + 1, len(a)):
                if a[y] != 0 and a[x] > a[y]:
                    count = count + 1
        if count == 0:
            return True
        if (count % 2) == 0:
            return True
        else:
            return False

    def all_solvable_children(self):
        MOVE = {"U": [-1, 0], "D": [1, 0], "L": [0, -1], "R": [0, 1]}
        children = []
        x0 = self.pos[0][0]
        y0 = self.pos[0][1]
        if x0 < 1:
            MOVE.pop("U")
        if x0 > 1:
            MOVE.pop("D")
        if y0 < 1:
            MOVE.pop("L")
        if y0 > 1:
            MOVE.pop("R")
        for dirc in MOVE:
            new_grid = self.grid.copy()
            i = MOVE[dirc][0]
            j = MOVE[dirc][1]
            new_grid[x0][y0], new_grid[x0 + i][y0 + j] = new_grid[x0 + i][y0 + j], new_grid[x0][y0]
            child = State(new_grid, self, dirc)
            if child.sovable:
                children.append(child)
        return children



def IDA(current_state):
    global count_ida
    count_ida=count_ida+1
    global THRESHOLD,NEXT_THRS,PATH
    if current_state.h==0:
        return True
    if (current_state.f>THRESHOLD):
        if(current_state.f<NEXT_THRS):
            NEXT_THRS = current_state.f
        return False
    children=current_state.all_solvable_children()
    for child in children:
        if current_state.p is not None:
            if child.list!=current_state.p.list:
                if IDA(child):
                    PATH.append(child.direction)
                    return True
        else:
            if IDA(child):
                PATH.append(child.direction)
                return True
    return False


def solution(puzzle):
    global GOAL,GOAL_POS,THRESHOLD,NEXT_THRS,PATH
    GOAL_POS = get_pos(GOAL)
    puzzle=numpy.array(puzzle)
    initial_state = State(puzzle, None, None)
    THRESHOLD=initial_state.h
    PATH = []
    if not initial_state.sovable:
        return PATH
    while not IDA(initial_state):
        PATH=[]
        if(NEXT_THRS!=float('Inf')):
            THRESHOLD=NEXT_THRS
            NEXT_THRS=float('Inf')
        else:
            break
    PATH.reverse()
    return PATH


def main():
    m=0
    time=0
    step=0
    for n in range(100):
        numbers = [0, 1, 2, 3, 4, 5, 6, 7, 8]
        puzzle = [[0, 0, 0],[0,0,0],[0,0,0]]
        for i in range(3):
            for j in range(3):
                random.shuffle(numbers)
                puzzle[i][j] = numbers[0]
                del numbers[0]
        # puzzle = [[8, 6, 7], [2, 5, 4], [3, 0, 1]]
        start = clock()
        path = solution(puzzle)
        finish = clock()
        time=time+(finish-start)
        step=step+len(path)
        if len(path)!=0:
            m+=1
            # print("solvable puzzle: ",m)
    print("solvable puzzle: ", m)
    print("avg time: ",time/m,"avg steps: ",step/m)
    print("include unsolvable: ",time/100,step/100)

main()








