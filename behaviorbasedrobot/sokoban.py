from collections import deque

board = []
size = 0

def init(board_string):
    global board, size
    temp = list(filter(None, board_string.splitlines()))
    size = len(temp[0])
    map_get_fixed_elements = {' ':' ', '.': '.', '@':' ', 'X':'#', '$':' ', '*':'.'}
    board =  [['' for i in range(size)] for j in range(size)]
    robot_position = ()
    can_positions = []

    for r, row in enumerate(temp):
        for c, character in enumerate(row):
            board[r][c] = map_get_fixed_elements[character] 
            if character == '@':
                robot_position = (c,r)
            if character == '$' or character == '*':
                can_positions.append((c,r))
    print('\n'.join([''.join(['{:2}'.format(item) for item in row]) for row in board]))
    print(robot_position)
    print(can_positions)

    queue = deque([(robot_position, can_positions, "")])
    visited = {(robot_position,str(can_positions))}
    return queue, visited

def push_can(x, y, dx, dy, can_pos):
    global board
    if board[y+dy][x+dx] == '#' or (x+dx,y+dy) in can_pos:
        return None
    else:
        can_pos.remove((x,y))
        can_pos.append((x+dx,y+dy))
        return can_pos

def is_solved(can_pos):
    global board, size
    for r in range(size):
        for c in range(size):
            if board[r][c] == '.':
                if (c,r) not in can_pos:
                    return False
    return True

def solve(queue, visited):
    global board
    directions = ((0, -1, "up ", "UP "), ( 1, 0, "right ", "RIGHT "), (0,  1, "down ", "DOWN "), (-1, 0, "left ", "LEFT "))
    while queue:
        robot_pos, can_positions, moves = queue.popleft()
        
        for dir in directions:
            x, y = robot_pos
            can_pos = can_positions[:] #copy list
            dx, dy = dir[0], dir[1] #the change in the x and y direction
            x = x+dx #update x
            y = y+dy #update y

            if board[y][x] == '#': #This position is out of the map
                continue
            elif (x,y) in can_pos: #can in this position
                new_can_pos = push_can(x, y, dx, dy, can_pos)
                if new_can_pos and ((x,y),str(new_can_pos)) not in visited: #fail if not possible to move the can or tried before
                    if is_solved(new_can_pos): #check if the new position of the cans is a solution
                        return moves + dir[2]
                    else: #add the new position of bot and cans to the queue and to visited
                        visited.add(((x,y), str(new_can_pos)))
                        queue.append(((x,y), new_can_pos, moves + dir[2]))
            else: #move to new position
                if ((x,y), str(can_pos)) not in visited: #fail if move is tried before
                    #add the new position of bot to the queue and to visited
                    visited.add(((x,y), str(can_pos)))
                    queue.append(((x,y), can_pos, moves + dir[2]))
    return "No solution"

board = """
XXXXXXXXX
X@     .X
X X X X X
X  $    X
X X X X X
X. $    X
X X X X X
X  $   .X
XXXXXXXXX"""

def translate_solution(str):
    solution = str.split()
    route = ""
    i = 0
    while (i<=len(solution)):
        if i == len(solution)-1:
            break
        if (solution[i] == solution[i+1]):
            route = route + solution[i] + " " 
            i = i+2
        else:
            route = route + solution[i] + " " 
            i = i+1
    return route + solution[len(solution)-1]

queue, visited = init(board)
solution = solve(queue, visited)
print(solution + '\n')
print(translate_solution(solution) + '\n')
