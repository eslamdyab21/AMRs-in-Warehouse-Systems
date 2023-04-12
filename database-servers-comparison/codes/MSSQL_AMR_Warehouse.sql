/*
To use MSSQL here: sqlcmd -S servername -E
    -- E for Windows Authentication
To show statistics of time: SET STATISTICS TIME ON;
*/

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

CREATE DATABASE AMR_Warehouse;
GO

USE AMR_Warehouse;
GO

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
    UNIQUE (CurrentLocationX, CurrentLocationY), 
    UNIQUE (NextLocationX, NextLocationY)
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* For the admin */

CREATE TABLE Notifications(
    NotificationID INT IDENTITY NOT NULL PRIMARY KEY, -- IDENTITY to be Auto-increment column
    Notification VARCHAR(255),
    Date DATETIME NOT NULL
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
This trigger does 2 tasks:
1. Add a notification for each new order
2. Decrease number of items in stock in products table
*/

GO
CREATE TRIGGER NewOrder ON Orders AFTER INSERT
AS
BEGIN
    DECLARE @shelfId VARCHAR(5);
    DECLARE @productId VARCHAR(5);
    DECLARE @quantity INT;

    SELECT @productId = ProductID FROM inserted;
    SELECT @shelfId = ShelfID FROM Shelves WHERE ProductID = @productId;
    SELECT @quantity = Quantity FROM inserted;

    INSERT INTO Notifications(Notification, Date)
        VALUES(CONCAT('A new product ', @productId, ' is ordered from shelf ', @shelfId, '.'), GETDATE());

    UPDATE Products SET ItemsInStock = ItemsInStock - @quantity WHERE ProductID = @productId;
    UPDATE Shelves SET HavingOrder = 1 WHERE ShelfID = @shelfId;
END
GO

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- This trigger does one task which is making the ShelfID in the Robots table becomes NULL once the HavingOrder state becomes 0

GO
CREATE TRIGGER CheckStates ON Robots INSTEAD OF UPDATE
AS
BEGIN
    DECLARE @currentState INT;
    DECLARE @robotId VARCHAR(5);

    SELECT @currentState = HavingOrder FROM inserted;
    SELECT @robotId = RobotID FROM inserted;

    IF @currentState = 0
    BEGIN
        UPDATE Robots SET ShelfID = NULL WHERE RobotID = @robotId;
    END
END
GO

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- This trigger does one task which is calculating the cost of new orders by multiplying the price of the product by the order's quantity

GO
CREATE TRIGGER CheckCost ON Orders INSTEAD OF INSERT
AS
BEGIN
    DECLARE @product_price INT;
    DECLARE @quantity INT;
    
    SELECT @product_price = P.Price FROM Products AS P INNER JOIN inserted AS I ON P.ProductID = I.ProductID;
    SELECT @quantity = Quantity FROM inserted;

    UPDATE Orders SET Cost = @product_price * @quantity;
END
GO

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------