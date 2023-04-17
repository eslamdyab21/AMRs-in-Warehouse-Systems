----------------------------------------------------------------------------------------------------------------------------

-- This database is just for testing purposes
-- It has no restrictions or any type of constraints
-- All columns are just defined by their data types, and all tables are just defined by their primary and foreign keys.

----------------------------------------------------------------------------------------------------------------------------

CREATE DATABASE testing_AMRs;

----------------------------------------------------------------------------------------------------------------------------

-- Interacting with the website

CREATE TABLE Users(
    UserID VARCHAR(5) PRIMARY KEY,      -- U1, U2, U3, ...
    Email VARCHAR(50),
    Password VARCHAR(50),
    FullName VARCHAR(20),
    Gender VARCHAR(10)
);

CREATE TABLE Products(
    ProductID VARCHAR(5) PRIMARY KEY,       -- P1, P2, P3, ...
    Price INT,
    ItemsInStock INT
);

CREATE TABLE Orders(
    OrderID VARCHAR(5) PRIMARY KEY,     -- O1, O2, O3, ...
    UserID VARCHAR(5),
    ProductID VARCHAR(5),
    Quantity INT,
    Cost DECIMAL(6, 2),
    DateTime DATETIME,

    -- Creating relations
    FOREIGN KEY (UserID) REFERENCES Users(UserID),
    FOREIGN KEY (ProductID) REFERENCES Products(ProductID)

);

----------------------------------------------------------------------------------------------------------------------------

-- Interacting with the warehouse

CREATE TABLE Shelves(
    ShelfID VARCHAR(5) PRIMARY KEY,     -- S1, S2, S3, ...
    LocationX INT,
    LocationY INT,
    ProductID VARCHAR(5), -- The product that the shelf stores
    HavingOrder INT,

    -- Creating relations
    FOREIGN KEY (ProductID) REFERENCES Products(ProductID)
);

CREATE TABLE Robots(
    RobotID VARCHAR(5) PRIMARY KEY,     -- R1, R2, R3, ...
    Speed INT,
    BatteryLife INT,

    -- Locations
    CurrentLocationX INT,
    CurrentLocationY INT,
    NextLocationX INT,
    NextLocationY INT,

    -- Information about the shelf
    ShelfID VARCHAR(5),

    -- Robot's states
    HavingOrder INT, -- 0: Empty, 1: Occupied
    Moving INT, -- 0: Not moving, 1: Moving

    -- Creating relations
    FOREIGN KEY (ShelfID) REFERENCES Shelves(ShelfID)
);

----------------------------------------------------------------------------------------------------------------------------
