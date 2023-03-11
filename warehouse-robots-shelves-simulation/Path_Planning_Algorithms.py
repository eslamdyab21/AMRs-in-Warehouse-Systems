import heapq


class Algorithms():
    def __init__(self):
        pass
    

    def heuristic(self, a, b):
        # return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
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

                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)

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