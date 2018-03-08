import heapq
import copy
import math
from time import clock
import numpy
import random

global GOAL, OPEN, CLOSE,GOAL_POS,MOVE_CASE
GOAL = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]
CLOSE = {}
OPEN=[]
MOVE_CASE={
            (0,0):[("D",(1,0)),("R",(0,1))],
            (0,1):[("D",(1,1)),("L",(0,0)),("R",(0,2))],
            (0,2):[("D",(1,2)),("L",(0,1))],
            (1,0):[("U",(0,0)),("D",(2,0)),("R",(1,1))],
            (1,1):[("U",(0,1)),("D",(2,1)),("L",(1,0)), ("R",(1,2))],
            (1,2): [("U",(0,2)),("D",(2,2)), ("L",(1,1))],
            (2,0):[("U",(1,0)),("R",(2,1))],
            (2,1):[("U",(1,1)),("L",(2,0)),("R",(2,2))],
            (2,2):[("U",(1,2)),("L",(2,1))],
           }


def get_pos(grid):
    pos = numpy.zeros((9,2),int)
    for i in range(3):
        for j in range(3):
            num=grid[i][j]
            pos[num][0] = i
            pos[num][1]=j
    return pos


def solvable(puzzle):
    puzzle=numpy.array(puzzle)
    a = numpy.reshape(puzzle,9).tolist()
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


def manhattan(grid_pos):
    distance = 0
    for num in range(1,9):
        # print(num,grid_pos)
        i,j=grid_pos[num][0],grid_pos[num][1]
        x,y=GOAL_POS[num][0],GOAL_POS[num][1]
        distance = distance + int(math.fabs(i - x) + math.fabs(j - y))
    return distance


def all_solvable_children(grid,grid_pos):
    global MOVE_CASE
    # MOVE = {"U": [-1, 0], "D": [1, 0], "L": [0, -1], "R": [0, 1]}
    children = []
    x0 = grid_pos[0][0]
    y0 = grid_pos[0][1]
    directions=MOVE_CASE[x0,y0]
    new_grid = copy.deepcopy(grid)
    for direc,pos in directions:
        i = pos[0]
        j = pos[1]
        new_grid[x0][y0], new_grid[i][j] = new_grid[i][j], new_grid[x0][y0]
        if solvable(new_grid):
            children.append([new_grid, direc])
        new_grid = copy.deepcopy(grid)
    return children


def solution(puzzle):
    global OPEN, CLOSE,GOAL_POS
    CLOSE={}
    OPEN=[]
    GOAL_POS = get_pos(GOAL)
    if solvable(puzzle):
        puzzle_pos=get_pos(puzzle)
        g=0
        h=manhattan(puzzle_pos)
        heapq.heappush(OPEN,(h+g,(puzzle,None,None,g,h,puzzle_pos)))
        count_it=0
        solution=""
        last_state=[]
        while len(OPEN)>0:
            count_it+=1
            current=heapq.heappop(OPEN)[1]
            current_grid=current[0]
            current_grid_pos=current[5]
            g=current[3]
            h=current[4]
            if h==0:
                last_state=current
                break
            else:
                children = all_solvable_children(current_grid,current_grid_pos)
                for child,direction in children:
                    if numpy.array(child).tostring() in CLOSE:
                        continue
                    child_pos = get_pos(child)
                    child_h=manhattan(child_pos)
                    heapq.heappush(OPEN,(child_h+g+1,(child,current,direction,g+1,child_h,child_pos)))
                    CLOSE[numpy.array(child).tostring()] = True

        while last_state is not None:
            direction=last_state[2]
            if direction:
                solution=last_state[2]+solution
                last_state=last_state[1]
            else:
                break

        # MOVE = {"U": [-1, 0], "D": [1, 0], "L": [0, -1], "R": [0, 1]}    # check the result
        # x0,y0=puzzle_pos[0][0],puzzle_pos[0][1]
        # for ind in range(len(solution)):
        #     i,j=MOVE[solution[ind]][0]+x0,MOVE[solution[ind]][1]+y0
        #     puzzle[x0][y0], puzzle[i][j] = puzzle[i][j], puzzle[x0][y0]
        #     x0,y0=i,j
        #     print(solution[ind],x0,y0,puzzle)
        return solution
    else:
        solution = ""
        return solution


def main():
    m=0
    time=0
    step=0

    time2=0
    step2=0

    for n in range(100):
        numbers = [0, 1, 2, 3, 4, 5, 6, 7, 8]
        puzzle = [[0, 0, 0],[0,0,0],[0,0,0]]
        for i in range(3):
            for j in range(3):
                random.shuffle(numbers)
                puzzle[i][j] = numbers[0]
                del numbers[0]
        start = clock()
        path=solution(puzzle)
        finish = clock()
        time=time+(finish-start)
        step=step+len(path)


        if len(path)!=0:
            m+=1

    print("solvable puzzle: ", m)
    print("avg time: ",time/m,"avg steps: ",step/m)
    print("include unsolvable: ",time/100,step/100)
    print(path,path2)

main()



