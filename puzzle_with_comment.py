import heapq
import copy
import numpy
import random
from time import clock


# get the position of 0 in the grid
def get_pos0(grid):
    for i in range(9):
        if grid[i]==0:
            return i


# check the grid is solvable or not
def solvable(grid):
    count = 0
    for x in range(9):
        for y in range(x + 1,9):
            if grid[y] != 0 and grid[x] > grid[y]:
                count += 1
    # count the number of inversions in the grid, odd->unsolvable even->solvable
    if count is 0:
        return True
    if (count % 2) is 0:
        return True
    else:
        return False


# calculate the manhattan distance of the whole grid
def manhattan(grid):
    distance = 0
    for num in range(9):
        if grid[num]>0:
            a = abs(grid[num] - 1 - num)
            b = int(a / 3) + (a % 3)  # vertical distance= a/3  horizon distance =a%3
            distance += b
    return distance


# For one state,it returns all the solvable children , the directions and the locations of zero
def all_solvable_children(grid,pos0):

    # position of zero determines the move case
    MOVE_CASE = {
        0: [("D", 3), ("R", 1)],  # if zero in position 0 it can  move down to pos3 or move right to pos1
        1: [("D", 4), ("L", 0), ("R", 2)],
        2: [("D", 5), ("L", 1)],  # if zero in position 2 it can  move down to pos5 or move left to pos1
        3: [("U", 0), ("D", 6), ("R", 4)],
        4: [("U", 1), ("D", 7), ("L", 3), ("R", 5)],
        5: [("U", 2), ("D", 8), ("L", 4)],
        6: [("U", 3), ("R", 7)],
        7: [("U", 4), ("L", 6), ("R", 8)],
        8: [("U", 5), ("L", 7)],
    }
    children = []

    directions=MOVE_CASE[pos0]
    new_grid = grid.copy()  # copy the value of the parent grid
    for i in directions:
        new_grid[pos0], new_grid[i[1]]=new_grid[i[1]],new_grid[pos0]  # switch the position of zero
        if solvable(new_grid):
            children.append([new_grid, i[0],i[1]])
        new_grid = copy.deepcopy(grid)  # copy the value of parent grid again and go next direction
    return children


def solution(puzzle):
    # initialization
    VISITED={}   # a dictionary used to record the states that have been visited.
    OPEN=[]      # a priority queue
    route = ""   # empty route
    last_state = None

    # convert the puzzle to a one dimension list, and mark it as visited
    puzzle=numpy.array(puzzle)
    VISITED[puzzle.tobytes()] = True
    initial=numpy.reshape(puzzle,9).tolist()

    # check the initial state is solvable or not
    if solvable(initial):
        # if it is solvable, push it into the priority queue
        g=0                     # g is the number of the steps before reaching this state
        h=manhattan(initial)    # the heuristic function using manhattan distance
        pos0=get_pos0(initial)  # get the location of zero in the grid
        heapq.heappush(OPEN,(h+g,(initial,"",g,h,pos0)))
        # the first element is the value of cost function f=g+h
        # the second element is a tuple (grid, the route it already went through, g,h,pos0)
        # the heapq.heappush() will arrange the the OPEN list as a MIN heap according to the value of f

        while len(OPEN)>0:
            # pop the state which has minimum f value
            current=heapq.heappop(OPEN)[1]
            current_grid=current[0]  # the grid of current state
            path=current[1]          # the route it already went through
            g=current[2]             # the number of the steps before reaching this state
            h=current[3]            # the heuristic function using manhattan distance
            current_pos0=current[4] # the location of zero in the grid

            # goal test
            if h is 0:              # when the heuristic function is 0, all number in right place
                last_state=current  # record the state which reaches the goal
                break

            else:
                # find all the solvable children with direction and location of zero
                children = all_solvable_children(current_grid,current_pos0)
                for child,direction,pos0 in children:
                    # if the child is already visited, just ignore it
                    if numpy.array(child).tobytes() in VISITED:
                        continue
                    # calculate the manhattan distance for this child
                    # add this child to the OPEN and mark it as visited
                    child_h = manhattan(child)
                    heapq.heappush(OPEN,(child_h+g+1,(child,path+direction,g+1,child_h,pos0)))
                    VISITED[numpy.array(child).tobytes()] = True

        # if we can find a state reaching the goal,then we can get the route
        if last_state is not None:
            route=last_state[1]

        return route
    else:
        route = ""
        return route


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

        start = clock()
        route=solution(puzzle)
        finish = clock()
        time=time+(finish-start)
        step=step+len(route)

        if len(route)!=0:
            m+=1

    print("solvable puzzle: ", m)
    print("avg time: ",time/m,"avg steps: ",step/m)
    print("include unsolvable: ",time/100,step/100)

main()

