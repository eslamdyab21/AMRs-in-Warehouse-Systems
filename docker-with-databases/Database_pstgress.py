import psycopg2
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

        ENV_POSTGRES_USER = os.getenv('POSTGRES_USER')
        ENV_POSTGRES_PASSWORD = os.getenv('POSTGRES_PASSWORD')
        ENV_POSTGRES_HOST = os.getenv('POSTGRES_HOST')
        ENV_POSTGRES_DATABASE = os.getenv('POSTGRES_DATABASE')
        ENV_POSTGRES_PORT = os.getenv('POSTGRES_PORT')
        self.connection = psycopg2.connect(database=ENV_POSTGRES_DATABASE,
                            user=ENV_POSTGRES_USER,
                            password=ENV_POSTGRES_PASSWORD,
                            host=ENV_POSTGRES_HOST,
                            port=ENV_POSTGRES_PORT)

        self.cursor = self.connection.cursor()
        self.logger.log(f'Database : connect_to_db : {time.time()-start_time} --> ' + "Connection is done")
        # self.cursor.execute(f"""USE {ENV_POSTGRES_DATABASE}""")
        self.logger.log(f'Database --> ' + f"{ENV_POSTGRES_DATABASE} Database is in use")


    def query_recived_order_shelfs_id(self):
        """
        query_recived_order_shelfs_id function queries the IDs of the shelves whose products are ordered by customers
        those shelves are defined by their state 'HavingOrder', this state = 1 if the shelf's product is ordered
        """
        start_time = time.time()

        # This query returns a list of all the shelves which their products are ordered
        query_shelves_ids = (
            """
                SELECT ShelfID FROM Shelves WHERE numoforders > 0;
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