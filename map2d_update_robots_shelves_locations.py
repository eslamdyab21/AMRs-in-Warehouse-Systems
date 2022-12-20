import pandas as pd
import numpy as np
import string


class Map2D():
    def __init__(self, size_x, size_y):
        self.size_x = size_x
        self.size_y = size_y
        self.map = np.zeros((size_x, size_y))
        self.map = self.map.astype('int')
        self.map = self.map.astype('object')




    def show_map(self):
        letters_seqence = string.ascii_uppercase[0:self.size_x]
        print(pd.DataFrame(self.map,columns=list(letters_seqence),index=list(letters_seqence)))
            
        print('---------------------------------------')