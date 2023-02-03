import mysql.connector as connector

class Database():
    """
    Write and query updated data from the database

    :param robots: (list) list of robots objects in the warehouse [robot1, robot2,....., robotn]
    :param shelves: (list) list of shelves objects in the warehouse [shelf1, shelf2,....., shelfn]
    :param map_size: (list) [map_size_x, map_size_y]
    """

    def __init__(self):
        #self.min_cost = map_size[0]
        self.connect_to_db()


    def query_shelf_id(self):
        query_notifications = (
            """
                SELECT SUBSTRING_INDEX(Notification, ' ', -1) as ShelfID, DateTime FROM Notifications ORDER BY NotificationID DESC LIMIT 1;
            """
        )

        self.cursor.execute(query_notifications)
        results = self.cursor.fetchall()[0]
        shelfId, time = results
        # print("Shelf ID = {} at time {}".format(shelfId, time))


    def connect_to_db(self):
        self.connection = connector.connect(
            user="mennatallah",
            password="Mmeenna71@SQL",
            port=3306,
            host="192.168.1.10",
            database="amr_warehouse"
        )
        self.cursor = self.connection.cursor()
        print("Connection is done")
        self.cursor.execute("""USE amr_warehouse""")
        print("Database is in use")

    def write_to_db(self, id, object):
        if id[0] == 'R':
            robot = object
            robot_parameters = (id, robot.speed, robot.prev_location[0], robot.prev_location[1], robot.prev_location[0], robot.prev_location[1], 'None', 0, robot.battery_precentage)
            write_to_robots = (
                """
                    INSERT INTO Robots(RobotID, Speed, CurrentLocationX, CurrentLocationY, NextLocationX, NextLocationY, ShelfID, CostToShelf)
                    VALUES(%s, %s, %s, %s, %s, %s, %s, %s)
                """
            )
            # The execution
            self.cursor.execute(write_to_robots, robot_parameters)
            print("A robot is added")

            battery = (id, robot.battery_precentage)
            write_to_robot_health = (
                """
                    INSERT INTO RobotHealth(RobotID, BatteryLife)
                    VALUES(%s, %s)
                """
            )
            self.cursor.execute(write_to_robot_health, battery)
            print("A robot's health is added")

        else:
            shelf = object
            shelf_parameters = (id, shelf.prev_location[0], shelf.prev_location[1], 'P2')
            # The query we'll execute
            write_to_shelves = (
                """
                    INSERT INTO Shelves(ShelfID, LocationX, LocationY, ProductID)
                    VALUES(%s, %s, %s, %s)
                """
            )
            # The execution
            self.cursor.execute(write_to_shelves, shelf_parameters)

    def update_db(self, table, id, parameters):
        if table == 'Robots' or table == "RobotHealth":
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
            ).format(table, column, value, primary_key, id)
            self.cursor.execute(update_robots)
        self.connection.commit()