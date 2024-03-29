import mysql.connector as connector
from dotenv import load_dotenv
import time
import os


class Database():
    """
    This class is responsible of the following tasks:
        1. connecting to database
        2. adding new objects to the database
        3. update existing objects
        4. querying information from the database

    :param logger: object from Logger class to log debugging and info data
    """

    def __init__(self, logger):
        #self.min_cost = map_size[0]
        self.logger = logger
        self.connect_to_db()



    def connect_to_db(self):
        """
        connect_to_db function is responsible of establishing the connection between the database and this python code
        using a cursor and defining the database which we need to be in use
        """
        
        start_time = time.time()

        load_dotenv()
        ENV_MYSQL_USER = os.getenv('MYSQL_USER')
        ENV_MYSQL_PASSWORD = os.getenv('MYSQL_PASSWORD')
        ENV_MYSQL_HOST = os.getenv('MYSQL_HOST')
        ENV_MYSQL_DATABASE = os.getenv('MYSQL_DATABASE')

        self.connection = connector.connect(
            user=ENV_MYSQL_USER,
            password=ENV_MYSQL_PASSWORD,
            port=3306,
            host=ENV_MYSQL_HOST,
            database=ENV_MYSQL_DATABASE
        )

        self.cursor = self.connection.cursor()
        self.logger.log(f'Database : connect_to_db : {time.time()-start_time} --> ' + "Connection is done")
        self.cursor.execute(f"""USE {ENV_MYSQL_DATABASE}""")
        self.logger.log('Database --> ' + "testing_AMRs Database is in use")


    def query_recived_order_shelfs_id(self):
        """
        query_recived_order_shelfs_id function queries the IDs of the shelves whose products are ordered by customers
        those shelves are defined by their state 'HavingOrder', this state = 1 if the shelf's product is ordered
        """
        start_time = time.time()

        # This query returns a list of all the shelves which their products are ordered
        query_shelves_ids = (
            """
                SELECT ShelfID FROM Shelves WHERE HavingOrder = 1;
            """
        )

        self.cursor.execute(query_shelves_ids)
        shelves_recived_order = self.cursor.fetchall()

        shelves_id_recived_order_list = []
        for shelf_id in shelves_recived_order:
            shelf_id, = shelf_id
            shelves_id_recived_order_list.append(shelf_id)

        self.logger.log(f"Database : query_recived_order_shelfs_id : {time.time()-start_time} --> Shelves that have recived an order: " + str(shelves_id_recived_order_list))
        
        return shelves_id_recived_order_list


    def write_to_db(self, id, object):
        """
        write_to_db function is responsible of adding new objects to the database whether this object is a robot or a shelf
        and for both, all the information required from the database for the object must be added
        
        this function defines the object to be added by the first letter of its ID (R: Robot, S: Shelf)
        """

        start_time = time.time()

        if id[0] == 'R':
            robot = object
            robot_parameters = (id, robot.speed, robot.battery_precentage, robot.prev_location[0], robot.prev_location[1], robot.current_location[0], robot.current_location[1], 'None')
            write_to_robots = (
                """
                    INSERT INTO Robots(RobotID, Speed, BatteryLife, CurrentLocationX, CurrentLocationY, NextLocationX, NextLocationY, ShelfID)
                    VALUES(%s, %s, %s, %s, %s, %s, %s, %s)
                """
            )
            self.cursor.execute(write_to_robots, robot_parameters)
            self.logger.log('Database --> ' + id + " robot is added with its health")

        else:
            shelf = object
            shelf_parameters = (id, shelf.prev_location[0], shelf.prev_location[1], shelf.id, shelf.recived_order_status)
            write_to_shelves = (
                """
                    INSERT INTO Shelves(ShelfID, LocationX, LocationY, ProductID, HavingOrder)
                    VALUES(%s, %s, %s, %s, %s)
                """
            )
            self.cursor.execute(write_to_shelves, shelf_parameters)
            self.logger.log(f'Database : write_to_db : {time.time()-start_time} --> ' + id + " shelf is added")

        self.connection.commit()



    def update_db(self, table, id, parameters):
        """
        update_db function is responsible of updating any object in the database whether it is a robot or a shelf
        with any information (not necessary all of them) by receiving an argument: the name of the table
        which the object that needs to be updated in

        the information we need to update is sent to the function as a dictionary, its keys represent the columns' names
        and its values represent the data we need to update
        """
        start_time = time.time()
        

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
        self.logger.log(f'Database : update_db : {time.time()-start_time} --> ' + id + " is updated")