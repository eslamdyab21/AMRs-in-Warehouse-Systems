class Shelf():
    """
    Shelf class creates Shelves for the warehouse.

    :param id: (string) id for the shelf to distinguish between shelves 
    :param intial_location: (list) [x, y]
    :param map_size: (list) [map_size_x, map_size_y]
    """
    def __init__(self, logger, id, intial_location, map_size):
        self.logger = logger

        self.id = id # ShelfID
        self.recived_order_status = False 
        self.paired_with_robot_status = False
        self.paired_with_robot = None
        self.physically_connected_to_robot = None
        self.intial_location = intial_location
        self.map_size = map_size

        self.prev_location = intial_location
        self.current_location = intial_location #LocationX, LocationY
        self.locations = [self.prev_location, self.current_location]

        info = f" is created with intial_location={intial_location}"
        self.logger.log('Shelf --> ' + id + info)



    def move_forward(self):
        """
        move_forward function moves the shelf in the forward direction based on its 
        current location and orientation         
        """
        self.prev_location = self.current_location.copy()

        if (self.orientation == 'down') and (self.current_location[0] != self.map_size[1]):
            self.current_location[0] = self.current_location[0] + 1
            
        elif (self.orientation == 'up') and (self.current_location[0] != 0):
            self.current_location[0] = self.current_location[0] - 1

        elif (self.orientation == 'right') and (self.current_location[1] != self.map_size[1]):
            self.current_location[1] = self.current_location[1] + 1
        
        elif (self.orientation == 'left') and (self.current_location[1] != 0):
            self.current_location[1] = self.current_location[1] - 1
        

        self.locations = [self.prev_location, self.current_location]

        self.logger.log('Shelf --> ' + self.id + f" moved to location={self.current_location}")


