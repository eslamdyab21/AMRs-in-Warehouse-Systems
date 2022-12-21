class Robot():
    """
    Robot class creates robots for the warehouse.

    :param id: (string) id for the robot to distinguish between robots 
    :param intial_location: (list) [x, y]
    :param intial_orientation: (string) robot's intial_orientation (up, down, right, left)
    :param speed: (float) robot speed
    :param map_size: (list) [map_size_x, map_size_y]
    """
    def __init__(self, id, intial_location, intial_orientation, speed, map_size):
        
        self.id = id
        self.speed = speed
        # active_order_status: (boolean) if robot is delivering an order (paired with a shelf) it's true
        self.active_order_status = False 
        # paired_with_shelf_status: (boolean) if robot is paired with a shelf it's true
        self.paired_with_shelf_status = False
        self.paired_with_shelf = None
        self.battery_precentage = 100
        # cost: (float) robot's path cost from its current location to the shelf location
        self.cost = None
        self.orientation = intial_orientation
        self.map_size = map_size

        self.prev_location = intial_location
        self.current_location = intial_location
        self.locations = [self.prev_location, self.current_location]


    def move_forward(self):
        """
        move_forward function moves the robot in the forward direction based on its 
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


    def rotate_90_degree_clock_wise(self):
        """
        rotate_90_degree_clock_wise function rotates the robot's orientation based
        on the robot current orientation.
        """
        if (self.orientation == 'down'):
            self.orientation = 'left'

        elif (self.orientation == 'up'):
            self.orientation = 'right'
        
        elif (self.orientation == 'right'):
            self.orientation = 'down'
        
        elif (self.orientation == 'left'):
            self.orientation = 'up'

            


    def rotate_90_degree_anti_clock_wise(self):
        """
        rotate_90_degree_clock_wise function rotates the robot's orientation based
        on the robot current orientation.
        """
        if (self.orientation == 'down'):
                self.orientation = 'right'

        elif (self.orientation == 'up'):
            self.orientation = 'left'
        
        elif (self.orientation == 'right'):
            self.orientation = 'up'
        
        elif (self.orientation == 'left'):
            self.orientation = 'down'


