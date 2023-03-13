import numpy as np
import string
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import pandas as pd

class Map2D():
    """
    Map2D class creates the 2d map with 2d numpy array, and because it's based on numpy 2d array
    the location's coordinates are based on numpy convention in indexing.

    :param size_x: width of the 2d grid
    :param size_x: height of the 2d grid
    """
    def __init__(self, size_x, size_y):
        self.size_x = size_x
        self.size_y = size_y
        self.map = np.zeros((size_y, size_x))
        self.map = self.map.astype('int')
        self.map = self.map.astype('object')



    def update_objects_locations(self, object_locations):
        """
        update_objects_locations function updates the locations of objects(robots or shelves) in the 2 grid.
        
        :param object_locations: dictionary of objects, each object has 2 attributes, the object name
                                 and a list of the object previous location and current location. 
                                 example: dict {R1:[prev_location, crrent_location], R2:[prev_location, crrent_location],.., Rn:[prev_location, crrent_location]}
        """

        for object_id, locations in object_locations.items():
            prev_location = locations[0]
            crrent_location = locations[1]

            # erase prev location
            pos_x = prev_location[0]
            pos_y = prev_location[1]
            self.map[pos_x][pos_y] = 0


            # add new location
            pos_x = crrent_location[0]
            pos_y = crrent_location[1]
            self.map[pos_x][pos_y] = object_id



    def show_map(self):
        """
        show_map function prints the 2d grid to visualize the movement of objects.
        """
        letters_seqence_columns = string.ascii_uppercase[0:self.size_x]
        letters_seqence_rows = string.ascii_uppercase[0:self.size_y]
        print(pd.DataFrame(self.map,columns=list(letters_seqence_columns),index=list(letters_seqence_rows)))
            
        print('---------------------------------------')



    def show_astar_map(self, astart_map, start, goal, route):
        x_coords = []

        y_coords = []

        for i in (range(0,len(route))):
            
                x = route[i][0]

                y = route[i][1]

                x_coords.append(x)

                y_coords.append(y)

        # plot map and path

        # plt.subplots(figsize=(20,20))
    
        plt.ion()

        plt.imshow(astart_map, cmap=plt.cm.Dark2)


        plt.plot(y_coords,x_coords, color = "black")

        plt.scatter(goal[1],goal[0], marker = "*", color = "red", s = 300)
        plt.scatter(start[1],start[0], marker = "o", color = "yellow", s = 50)

        plt.show()
        plt.pause(1)