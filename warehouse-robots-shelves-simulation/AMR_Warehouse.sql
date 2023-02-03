-- To use MySQL here --
-- mysql -u username -p --

-- Need to remove CostToShelf

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- To enable or disable foreign key constraints: SET FOREIGN_KEY_CHECKS=0; or 1;
CREATE DATABASE AMR_Warehouse;

-- Interacting with the website --
-- ALL IS ADDED --

CREATE TABLE Users(
    UserID INT NOT NULL PRIMARY KEY AUTO_INCREMENT, -- No need to insert it manually
    Email VARCHAR(50) NOT NULL UNIQUE,
    Password VARCHAR(50) NOT NULL,
    FullName VARCHAR(20) NOT NULL,
    Gender VARCHAR(10) NOT NULL CHECK(Gender IN ('Male', 'Female'))
);

CREATE TABLE Products(
    ProductID VARCHAR(5) NOT NULL PRIMARY KEY,
    Price INT NOT NULL CHECK(Price > 0),
    ItemsInStock INT NOT NULL CHECK(ItemsInStock >= 0)
);

CREATE TABLE Orders(
    OrderID VARCHAR(5) NOT NULL PRIMARY KEY,
    UserID INT NOT NULL,
    ProductID VARCHAR(5) NOT NULL,
    Quantity INT NOT NULL CHECK(Quantity >= 0),
    Cost DECIMAL(6 , 2) NOT NULL CHECK(Cost >= 0),
    DateTime DATETIME NOT NULL,

    -- Creating relations
    FOREIGN KEY (UserID) REFERENCES Users(UserID),
    FOREIGN KEY (ProductID) REFERENCES Products(ProductID)

);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Interacting with the warehouse --
-- ALL IS ADDED --

CREATE TABLE Shelves(
    ShelfID VARCHAR(5) NOT NULL PRIMARY KEY,
    LocationX INT NOT NULL UNIQUE CHECK(LocationX BETWEEN 0 AND 20),
    LocationY INT NOT NULL UNIQUE CHECK(LocationY BETWEEN 0 AND 20),
    ProductID VARCHAR(5) NOT NULL, -- The product that the shelf stores

    -- Creating relations
    FOREIGN KEY (ProductID) REFERENCES Products(ProductID)
);

CREATE TABLE Robots(
    RobotID VARCHAR(5) NOT NULL PRIMARY KEY,
    Speed INT NOT NULL CHECK(Speed >= 0),

    -- Locations
    CurrentLocationX INT NOT NULL UNIQUE CHECK(CurrentLocationX BETWEEN 0 AND 20),
    CurrentLocationY INT NOT NULL UNIQUE CHECK(CurrentLocationY BETWEEN 0 AND 20),
    NextLocationX INT CHECK(NextLocationX BETWEEN 0 AND 20),
    NextLocationY INT CHECK(NextLocationY BETWEEN 0 AND 20),

    -- Information about the shelf
    ShelfID VARCHAR(5),
    CostToShelf INT NOT NULL CHECK(CostToShelf >= 0),

    -- Creating relations
    FOREIGN KEY (ShelfID) REFERENCES Shelves(ShelfID)
);

CREATE TABLE RobotHealth(
    RobotID VARCHAR(5) NOT NULL,
    BatteryLife INT NOT NULL CHECK(BatteryLife BETWEEN 0 AND 100)
);

CREATE TABLE States(
    RobotID VARCHAR(5) NOT NULL UNIQUE,
    HavingOrder INT NOT NULL CHECK(HavingOrder IN (0, 1)), -- 0: Empty, 1: Occupied
    Moving INT NOT NULL CHECK(Moving IN (0, 1)),

    -- Creating relations
    FOREIGN KEY (RobotID) REFERENCES Robots(RobotID)
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- For the developer --
-- ADDED --

CREATE TABLE Notifications(
    NotificationID INT NOT NULL PRIMARY KEY AUTO_INCREMENT,
    Notification VARCHAR(100) NOT NULL,
    DateTime DATETIME NOT NULL
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Trigger for notification a new order & decrease number of items in stock in products table (ADDED)
DELIMITER //
CREATE TRIGGER NewOrder AFTER INSERT ON Orders FOR EACH ROW
    BEGIN
        DECLARE shelfId VARCHAR(5);
        SELECT S.ShelfID INTO shelfId FROM Shelves AS S WHERE S.ProductID = NEW.ProductID;
        INSERT INTO Notifications(Notification, DateTime)
            VALUES(CONCAT('A new product is ordered from shelf ', shelfId), TIMESTAMP(NEW.DateTime));
        UPDATE Products AS P SET P.ItemsInStock = P.ItemsInStock - NEW.Quantity WHERE P.ProductID = NEW.ProductID;
    END //
DELIMITER ;

-- STOP HERE --
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Function to send an order to the robot with the minimum cost & minimum ID to go
CREATE FUNCTION RobotToGo()
    RETURNS VARCHAR(5) DETERMINISTIC
    RETURN(SELECT MIN(RobotID) FROM Robots WHERE CostToShelf = (SELECT MIN(CostToShelf) FROM Robots));

-- To call it: SELECT RobotToGo();

-- Trigger for deadlock state between 2 robots
DELIMITER //
CREATE TRIGGER DeadLock AFTER UPDATE ON Robots FOR EACH ROW
    BEGIN
        DECLARE deadlock VARCHAR(5);
        -- This CTE returns all the robots' IDs which will be at the same location
        WITH RobotsAtSameLocation AS
        (
            SELECT R1.RobotID FROM Robots AS R1
            INNER JOIN Robots AS R2
                ON R1.NextLocationX = R2.NextLocationX
                    AND R1.NextLocationY = R2.NextLocationY
            WHERE R1.RobotID <> R2.RobotID
        )

        -- For only 2 robots in the grid
        SELECT MAX(RobotID) INTO deadlock FROM RobotsAtSameLocation;
        UPDATE States SET Moving = 0 WHERE RobotID = deadlock;

        -- For more than 2 robots in the grid
        -- To loop over the CTE
        -- DECLARE NumberOfDeadlocks INT;
        -- SELECT COUNT(*) INTO NumberOfDeadlocks FROM RobotsAtSameLocation;
        -- CALL LoopOverStates(NumberOfDeadlocks);
    END//
DELIMITER ;

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
-- Things to do:
-- 1. Decrease the items in stock in Products table in the case of ordering an order (Done)
-- 2. Check the Cost in the Orders table that is equal to Quantity*Product.Price
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- LoopOverStates procedure to make all robots' states is deadlock except the one with minimum ID
DELIMITER //
CREATE PROCEDURE LoopOverStates(IN NIterations INT)
    BEGIN
        DECLARE inc INT;
        SET inc = 0;
        LABEL:
            WHILE inc <= NIterations DO
                UPDATE Robots SET Moving = 0 WHERE RobotID = CONCAT('R', inc);
                SET inc = inc + 1;
            END
        WHILE LABEL;
    END//
DELIMITER ;

-- Trigger for states with shelves
DELEMITER //
CREATE TRIGGER CheckStates AFTER INSERT ON Robots FOR EACH ROW
    BEGIN
        DECLARE currentState VARCHAR(10);
        SELECT HavingOrder INTO currentState FROM States;
        IF currentState = 0 THEN
            REPLACE INTO Robots SET RobotID = New.RobotID, ShelfID = NULL, CostToShelf = 0;
        END IF;
    END //
DELEMITER ;

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

