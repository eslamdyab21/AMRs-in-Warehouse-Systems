/*
To use PostgreSQL here: psql -U postgres
To show statistics of time: EXPLAIN ANALYZE query
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
    OrderDate DATE NOT NULL,
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
    UNIQUE (CurrentLocationX, CurrentLocationY), 
    UNIQUE (NextLocationX, NextLocationY)
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* For the admin */

CREATE TABLE Notifications(
    NotificationID SERIAL NOT NULL PRIMARY KEY,
    Notification VARCHAR(255),
    Date DATE NOT NULL
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
This trigger does 2 tasks:
1. Add a notification for each new order
2. Decrease number of items in stock in products table
*/

CREATE OR REPLACE FUNCTION new_order() RETURNS TRIGGER AS $$
DECLARE
    shelf_id VARCHAR(5);
    product_id VARCHAR(5);
BEGIN
    SELECT NEW.ProductID INTO product_id;
    SELECT S.ShelfID INTO shelf_id FROM Shelves AS S WHERE S.ProductID = NEW.ProductID;
    INSERT INTO Notifications(Notification, Date)
        VALUES(CONCAT('A new product ', product_id, ' is ordered from shelf ', shelf_id, '.'), NOW());
    UPDATE Products AS P SET ItemsInStock = ItemsInStock - NEW.Quantity WHERE P.ProductID = NEW.ProductID;
    UPDATE Shelves AS S SET HavingOrder = 1 WHERE S.ShelfID = shelf_id;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER NewOrder AFTER INSERT ON Orders
    FOR EACH ROW
    EXECUTE FUNCTION new_order();

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- This trigger does one task which is making the ShelfID in the Robots table becomes NULL once the HavingOrder state becomes 0

CREATE OR REPLACE FUNCTION check_states() RETURNS TRIGGER AS $$
DECLARE
    current_state INT;
BEGIN
    SELECT NEW.HavingOrder INTO current_state FROM Robots;
    IF current_state = 0 THEN
        NEW.ShelfID := NULL;
    END IF;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER CheckStates BEFORE UPDATE ON Robots
    FOR EACH ROW
    EXECUTE FUNCTION check_states();

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- This trigger does one task which is calculating the cost of new orders by multiplying the price of the product by the order's quantity

CREATE OR REPLACE FUNCTION check_cost() RETURNS TRIGGER AS $$
DECLARE
    product_price INT;
BEGIN
    SELECT P.Price INTO product_price FROM Products AS P WHERE P.ProductID = NEW.ProductID;
    NEW.Cost := product_price * NEW.Quantity;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER CheckCost BEFORE INSERT ON Orders
    FOR EACH ROW
    EXECUTE FUNCTION check_cost();

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
/*
Things to do:
-------------
1. (Done) Change DateTime in Orders table to OrderDate and Notifications table Date
2. (Done) Add a status to the orders table (Status) -> New, In progress, Completed (New by default)
3. Add a new trigger, once the shelf arrived the packaging area, the order would be marked as Completed
*/
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------