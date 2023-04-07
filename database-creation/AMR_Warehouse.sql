/*
To use MySQL here: mysql -u username -p
To enable or disable foreign key constraints: SET FOREIGN_KEY_CHECKS=0; or 1;
*/

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

CREATE DATABASE AMR_Warehouse;

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* Interacting with the website */

CREATE TABLE Users(
    UserID VARCHAR(5) NOT NULL PRIMARY KEY,
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
    UserID VARCHAR(5) NOT NULL,
    ProductID VARCHAR(5) NOT NULL,
    Quantity INT NOT NULL CHECK(Quantity > 0),
    Cost DECIMAL(6 , 2) CHECK(Cost >= 0),
    OrderDate DATETIME NOT NULL, -- DATE for postgres
    Status VARCHAR(10) CHECK(Status IN ('New', 'In progress', 'Completed')) DEFAULT 'New',

    -- Creating relations
    FOREIGN KEY (UserID) REFERENCES Users(UserID),
    FOREIGN KEY (ProductID) REFERENCES Products(ProductID)
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* Interacting with the warehouse */

CREATE TABLE Shelves(
    ShelfID VARCHAR(5) NOT NULL PRIMARY KEY,
    LocationX INT NOT NULL CHECK(LocationX BETWEEN 0 AND 20),
    LocationY INT NOT NULL CHECK(LocationY BETWEEN 0 AND 20),
    ProductID VARCHAR(5), -- The product that the shelf stores
    HavingOrder INT NOT NULL CHECK(HavingOrder IN (0, 1)), -- 0: Empty, 1: Occupied

    -- Creating relations
    FOREIGN KEY (ProductID) REFERENCES Products(ProductID)
);

CREATE TABLE Robots(
    RobotID VARCHAR(5) NOT NULL PRIMARY KEY,
    Speed INT NOT NULL CHECK(Speed >= 0),
    BatteryLife INT NOT NULL CHECK(BatteryLife BETWEEN 0 AND 100),

    -- Locations
    CurrentLocationX INT NOT NULL CHECK(CurrentLocationX BETWEEN 0 AND 20),
    CurrentLocationY INT NOT NULL CHECK(CurrentLocationY BETWEEN 0 AND 20),
    NextLocationX INT CHECK(NextLocationX BETWEEN 0 AND 20),
    NextLocationY INT CHECK(NextLocationY BETWEEN 0 AND 20),

    -- Information about the shelf
    ShelfID VARCHAR(5),

    -- Robot's states
    HavingOrder INT NOT NULL CHECK(HavingOrder IN (0, 1)), -- 0: Empty, 1: Occupied
    Moving INT NOT NULL CHECK(Moving IN (0, 1)), -- 0: Not moving, 1: Moving

    -- Creating relations
    FOREIGN KEY (ShelfID) REFERENCES Shelves(ShelfID),

    -- To add the constraint of unique location (x, y)
    UNIQUE KEY (CurrentLocationX, CurrentLocationY), 
    UNIQUE KEY (NextLocationX, NextLocationY)
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* For the admin */

CREATE TABLE Notifications(
    NotificationID INT NOT NULL PRIMARY KEY AUTO_INCREMENT, -- NotificationID SERIAL NOT NULL PRIMARY KEY -- for postgres
    Notification VARCHAR(255),
    Date DATETIME NOT NULL -- DATE for postgres
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
This trigger does 2 tasks:
1. Add a notification for each new order
2. Decrease number of items in stock in products table
*/

DELIMITER //
CREATE TRIGGER NewOrder AFTER INSERT ON Orders FOR EACH ROW
    BEGIN
        DECLARE shelfId VARCHAR(5);
        DECLARE productId VARCHAR(5);
        SELECT New.ProductID INTO productId;
        SELECT S.ShelfID INTO shelfId FROM Shelves AS S WHERE S.ProductID = NEW.ProductID;
        INSERT INTO Notifications(Notification, Date)
            VALUES(CONCAT('A new product ', productId, ' is ordered from shelf ', shelfId, '.'), NOW());
        UPDATE Products AS P SET P.ItemsInStock = P.ItemsInStock - NEW.Quantity WHERE P.ProductID = NEW.ProductID;
        UPDATE Shelves AS S SET S.HavingOrder = 1 WHERE S.ShelfID = shelfId;
    END //
DELIMITER ;

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- This trigger does one task which is making the ShelfID in the Robots table becomes NULL once the HavingOrder state becomes 0

DELEMITER //
CREATE TRIGGER CheckStates BEFORE UPDATE ON Robots FOR EACH ROW
    BEGIN
        DECLARE currentState INT;
        SELECT NEW.HavingOrder INTO currentState FROM Robots;
        IF currentState = 0 THEN
            SET NEW.ShelfID = NULL;
        END IF;
    END //
DELEMITER ;

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- This trigger does one task which is calculating the cost of new orders by multiplying the price of the product by the order's quantity

DELIMITER //
CREATE TRIGGER CheckCost BEFORE INSERT ON Orders FOR EACH ROW
    BEGIN
        DECLARE product_price INT;
        SELECT Price INTO product_price FROM Products AS P WHERE P.ProductID = NEW.ProductID;
        SET NEW.Cost = product_price * NEW.Quantity;
    END //
DELIMITER ;

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
/*
Things to do:
-------------
1. (Done) Change DateTime in Orders table to OrderDate and Notifications table Date
2. (Done) Add a status to the orders table (Status) -> New, In progress, Completed (New by default)
3. Add a new trigger, once the shelf arrived the packaging area, the order would be marked as Completed
*/
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------