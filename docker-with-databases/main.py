from Database_pstgress import Database
from Logger import Logger
import time

logger = Logger()

db = Database(logger)


for i in range(10):
    db.update_db(table="Shelves", id='S2', 
                parameters={"location_x":i+2, "location_y":i+3, "numoforders":i})

    db.update_db(table="Shelves", id='S1', 
                parameters={"location_x":i+4, "location_y":i+5, "numoforders":i+1})
    
    db.update_db(table="Robots", id='R2', 
                parameters={"currentlocation_x":i+6, "currentlocation_y":i+7, "batterypercentage":i*3})

    db.update_db(table="Robots", id='R1', 
                parameters={"currentlocation_x":i+8, "currentlocation_y":i+9, "batterypercentage":i*1})


    db.query_recived_order_shelfs_id()
    logger.log("------------------------breaker------------------------")
    logger.log(" ")


    time.sleep(1)