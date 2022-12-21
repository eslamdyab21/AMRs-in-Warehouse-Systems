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


    def min_cost_robots(self):
        """
        min_cost_robots function gets robots with minimum cost to shelves, for each shelf
        we iterate over all robots that are not paired with other shelves and compute the
        corresponding cost until the robot with minimum cost is found, this robot is 
        then appended to a list.
        """
        for shelf in self.shelvs:
            if shelf.active_order_status:   
                for robot in self.robots:
                    if robot.paired_with_shelf_status == False:
                        robot.active_order_status = False  
                        cost_vector = abs(robot.location - shelf.location)
                        cost = cost_vector[0] + cost_vector[1]
                        if cost < self.min_cost:
                            self.min_cost = cost
                            self.robot_with_min_cost = robot

                # this condition is checked for each shelf
                if self.robot_with_min_cost != None:
                    if self.robot_with_min_cost.active_order_status == False:
                        self.robot_with_min_cost.active_order_status = True
                        self.robot_with_min_cost.paired_with_shelf_status = True
                        self.robot_with_min_cost.paired_with_shelf = shlef

                        self.robots_with_min_cost_list.append(robot_with_min_cost)
