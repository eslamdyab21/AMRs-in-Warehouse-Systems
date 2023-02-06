import mysql.connector as connector


class Database():
    """
    Write and query updated data from the database

    :param robots: (list) list of robots objects in the warehouse [robot1, robot2,....., robotn]
    :param shelves: (list) list of shelves objects in the warehouse [shelf1, shelf2,....., shelfn]
    :param map_size: (list) [map_size_x, map_size_y]
    """

    def __init__(self, logger):
        #self.min_cost = map_size[0]
        self.logger = logger
        self.connect_to_db()
        



    def query_recived_order_shelfs_id(self):
        query_notifications = (
            """
                SELECT ShelfID FROM Shelves WHERE HavingOrder = 1;
            """
        )

        self.cursor.execute(query_notifications)
        shelves_recived_order = self.cursor.fetchall()

        shelves_id_recived_order_list = []
        for shelf_id in shelves_recived_order:
            shelf_id, = shelf_id
            shelves_id_recived_order_list.append(shelf_id)

        self.logger.log("Database --> Shelves that have recived an order: " + str(shelves_id_recived_order_list))
        
        return shelves_id_recived_order_list



    def connect_to_db(self):
        self.connection = connector.connect(
            user="dyab",
            password="@BMW123bmw",
            port=3306,
            host="localhost",
            database="AMR_Warehouse"
        )

        self.cursor = self.connection.cursor()
        self.logger.log('Database --> ' + "Connection is done")
        self.cursor.execute("""USE AMR_Warehouse""")
        self.logger.log('Database --> ' + "AMR_Warehouse Database is in use")



    def write_to_db(self, id, object):
        if id[0] == 'R':
            robot = object
            robot_parameters = (id, robot.speed, robot.battery_precentage, robot.prev_location[0], robot.prev_location[1], robot.current_location[0], robot.current_location[1], 'None')
            write_to_robots = (
                """
                    INSERT INTO Robots(RobotID, Speed, BatteryLife, CurrentLocationX, CurrentLocationY, NextLocationX, NextLocationY, ShelfID)
                    VALUES(%s, %s, %s, %s, %s, %s, %s, %s)
                """
            )
            # The execution
            self.cursor.execute(write_to_robots, robot_parameters)
            self.logger.log('Database --> ' + id + " robot is added with its health")

        else:
            shelf = object
            shelf_parameters = (id, shelf.prev_location[0], shelf.prev_location[1], shelf.id, self.recived_order_status)
            # The query we'll execute
            write_to_shelves = (
                """
                    INSERT INTO Shelves(ShelfID, LocationX, LocationY, ProductID, HavingOrder)
                    VALUES(%s, %s, %s, %s, %s)
                """
            )
            # The execution
            self.cursor.execute(write_to_shelves, shelf_parameters)
            self.logger.log('Database --> ' + id + " shelf is added")

        self.connection.commit()



    def update_db(self, table, id, parameters):
        if table == 'Robots':
            primary_key = "RobotID"
        else:
            primary_key = "ShelfID"

        for column, value in parameters.items():
            update_robots = (
                """
                    UPDATE {}
                    SET {} = {}
                    WHERE {} = {}
                """
            ).format(table, column, value, primary_key, "'" + id + "'")
            
            self.cursor.execute(update_robots)
        self.connection.commit()
        self.logger.log('Database --> ' + id + " is updated")