import heapq
import numpy as np


class Algorithms():
    def __init__(self):
        self.direction = None
        self.prev_direction = None
        self.init = False
        self.counter = 0
    

    def heuristic(self, a, b, steps_score=False):
        
        if steps_score:
            
            current = a
            neighbor = b

            if self.init == False:
                self.prev_current = current
                self.init = True
                return abs((b[0] - a[0]) + abs((b[1] - a[1])))
            
            
            
            
            # if self.prev_current != current:
            # print('in self.prev_current != current')
            self.counter = self.counter + 1

            if self.counter >= 4:
                print('steps_score')
                print(self.prev_current,a,b)
                if self.prev_current[0] - current[0] == 0:
                    self.direction = 'horizontal'
                    if current[0] - neighbor[0] != 0:
                        cost = abs((b[0] - a[0]) + abs((b[1] - a[1]))) + 1
                    else:
                        cost = abs((b[0] - a[0]) + abs((b[1] - a[1])))


                elif self.prev_current[1] - current[1] == 0:
                    self.direction = 'vertical'
                    if current[1] - neighbor[1] != 0:
                        cost = abs((b[0] - a[0]) + abs((b[1] - a[1]))) + 1
                    else:
                        cost = abs((b[0] - a[0]) + abs((b[1] - a[1])))

                if self.counter % 4 == 0:
                    print('in counter')
                    self.prev_current = current
                    

                return cost
            
            else:
                return abs((b[0] - a[0]) + abs((b[1] - a[1])))
                
        return abs((b[0] - a[0]) + abs((b[1] - a[1])))
            



    def astar(self, grid, start, goal):
        start = tuple(start)
        goal = tuple(goal)
        array = grid

        # neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        neighbors = [(0,1),(0,-1),(1,0),(-1,0)]

        close_set = set()

        came_from = {}

        gscore = {start:0}

        fscore = {start:self.heuristic(start, goal)}

        oheap = []

        heapq.heappush(oheap, (fscore[start], start))
    

        while oheap:
            current = heapq.heappop(oheap)[1]
            # made it to the goal location
            if current == goal:
                data = []

                while current in came_from:
                    data.append(current)
                    current = came_from[current]

                route = data + [start]
                route = route[::-1]

                return route


            close_set.add(current)

            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor, steps_score=True)

                if 0 <= neighbor[0] < array.shape[0]:
                    if 0 <= neighbor[1] < array.shape[1]:                
                        if array[neighbor[0]][neighbor[1]] == 1:
                            continue

                    else:
                        # array bound y walls
                        continue

                else:
                    # array bound x walls
                    continue
    

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
    

                if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:

                    came_from[neighbor] = current

                    gscore[neighbor] = tentative_g_score

                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
    

        return False