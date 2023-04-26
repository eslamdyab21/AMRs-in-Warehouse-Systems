-- Inserting queries: The following queries has to be inserted in order

INSERT INTO Customers VALUES('C1', 'menna@gmail', 'mennapassword', 'Mennatallah Mamdouh', 'Female');

INSERT INTO Products VALUES('P1', 'Hair Shampoo', 250, 'It is a hair product. It contains Sulfates', 500, 'URL1', 'URL2', 'URL3'),
                            ('P2', 'Hair Conditioner', 300, 'It is a hair product. It controls frizzness', 500, 'URL1', 'URL2', 'URL3');

INSERT INTO Shelves(ShelfID, Location_X, Location_Y, ProductID, NumOfOrders)
VALUES('S1', 0, 5, 'P1', 0), ('S2', 0, 10, 'P2', 0);

INSERT INTO Orders_Details VALUES('O1', 'C1', NULL, NOW(), '+201234567890', 'Alexandria', 'Cash on Delivery', 'New');

INSERT INTO Orders VALUES('O1', 'P1', 5), ('O1', 'P2', 7);

INSERT INTO Robots(RobotID, Speed, BatteryPercentage, CurrentLocation_X, CurrentLocation_Y, isCharging, ShelfID)
VALUES('R1', 90, 100, 5, 5, False, 'S1'), ('R2', 80, 50, 10, 10, False, 'S2');