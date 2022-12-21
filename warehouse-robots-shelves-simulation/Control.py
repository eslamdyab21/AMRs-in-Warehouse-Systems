class Control():
    """
    Control class controls the robots movement, find the cost of each robot path to the shelf
    and decides which robot will move (lowest cost) and gives the steps needed to
    get to the shelf to the chosen robot.

    :param robots: (list) list of robots objects in the warehouse [robot1, robot2,....., robotn]
    :param shelves: (list) list of shelves objects in the warehouse [shelf1, shelf2,....., shelfn]
    :param map_size: (list) [map_size_x, map_size_y]
    :param map_size: (list) [map_size_x, map_size_y]
    """
    def __init__(self, robots, shelvs, map_size):

        self.robots = robots
        self.shelvs = shelvs
        self.map_size = map_size
        self.min_cost = None
        self.robots_with_min_cost_list = []
