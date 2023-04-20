/*
To use MySQL here: mysql -u username -p
To enable or disable foreign key constraints: SET FOREIGN_KEY_CHECKS=0; or 1;
To reset counting on auto-incremental columns: ALTER TABLE table_name AUTO_INCREMENT = 1;
To get information from any table in MySQL: desc table_name
*/

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

DROP DATABASE IF EXISTS AMR_Warehouse;

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

CREATE DATABASE IF NOT EXISTS AMR_Warehouse;

USE AMR_Warehouse;

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* Interacting with the website */

CREATE TABLE IF NOT EXISTS Customers(
    CustomerID VARCHAR(7),
    Email VARCHAR(50),
    Password VARCHAR(20),
    FullName VARCHAR(25),
    Gender ENUM('Male', 'Female'),

    -- Constraints
    CONSTRAINT PK_Customers PRIMARY KEY(CustomerID),
    CONSTRAINT unique_mail UNIQUE(Email)
);

CREATE TABLE IF NOT EXISTS Products(
    ProductID VARCHAR(7),
    ProductName VARCHAR(50) NOT NULL,
    Price DECIMAL(7, 2) NOT NULL CHECK(Price > 0),
    Description VARCHAR(255),
    ItemsInStock INT NOT NULL CHECK(ItemsInStock >= 0),
    Image_1 VARCHAR(255),
    Image_2 VARCHAR(255),
    Image_3 VARCHAR(255),

    -- Constraints
    CONSTRAINT PK_Products PRIMARY KEY(ProductID)
);

CREATE TABLE IF NOT EXISTS Orders(
    OrderID VARCHAR(7),
    CustomerID VARCHAR(7),
    TotalCost FLOAT CHECK(TotalCost >= 0) DEFAULT NULL,
    OrderDate TIMESTAMP DEFAULT NOW(),
    PhoneNumber VARCHAR(13),
    Address VARCHAR(255),
    PaymentMethod ENUM('Cash on Delivery', 'Credit Card', 'Paypal', 'Paytm'),
    OrderStatus ENUM('New', 'In progress', 'Completed') DEFAULT 'New',

    -- Constraints
    CONSTRAINT PK_Orders PRIMARY KEY(OrderID),
    CONSTRAINT FK_customers_in_orders FOREIGN KEY (CustomerID) REFERENCES Customers(CustomerID)
);

CREATE TABLE IF NOT EXISTS Orders_Details(
    OrderID VARCHAR(7),
    ProductID VARCHAR(7),
    Quantity INT CHECK(Quantity > 0),

    -- Constraints
    CONSTRAINT PK_Orders_Details PRIMARY KEY(OrderID, ProductID),
    CONSTRAINT FK_orders_in_orders_details FOREIGN KEY (OrderID) REFERENCES Orders(OrderID),
    CONSTRAINT FK_products_in_orders_details FOREIGN KEY (ProductID) REFERENCES Products(ProductID)
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* Interacting with the warehouse */

CREATE TABLE IF NOT EXISTS Shelves(
    ShelfID VARCHAR(7),
    Location_X INT NOT NULL CHECK(Location_X BETWEEN 0 AND 20),
    Location_Y INT NOT NULL CHECK(Location_Y BETWEEN 0 AND 20),
    ProductID VARCHAR(7), -- The product that the shelf stores
    isHavingOrder BOOL, -- 0: Empty, 1: Occupied

    -- Constraints
    CONSTRAINT PK_Shelves PRIMARY KEY(ShelfID),
    CONSTRAINT FK_products_in_shelves FOREIGN KEY (ProductID) REFERENCES Products(ProductID),
    CONSTRAINT unique_locations UNIQUE (Location_X, Location_Y)
);

CREATE TABLE IF NOT EXISTS Robots(
    RobotID VARCHAR(7),
    Speed DECIMAL(5, 2) CHECK(Speed >= 0),
    BatteryPercentage INT NOT NULL CHECK(BatteryPercentage BETWEEN 0 AND 100),
    CurrentLocation_X INT NOT NULL CHECK(CurrentLocation_X BETWEEN 0 AND 20),
    CurrentLocation_Y INT NOT NULL CHECK(CurrentLocation_Y BETWEEN 0 AND 20),
    isCharging BOOL,
    ShelfID VARCHAR(7), -- The shelf that the robot holds

    -- Constraints
    CONSTRAINT PK_Robotos PRIMARY KEY(RobotID),
    CONSTRAINT FK_shelves_with_robots FOREIGN KEY (ShelfID) REFERENCES Shelves(ShelfID),
    CONSTRAINT unique_locations UNIQUE (CurrentLocation_X, CurrentLocation_Y),
    CONSTRAINT unique_shelves UNIQUE(ShelfID)
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* For the admin */

CREATE TABLE IF NOT EXISTS Notifications(
    NotificationID INT AUTO_INCREMENT,
    Notification VARCHAR(255),
    NotificationDateTime TIMESTAMP DEFAULT NOW(),

    -- Constraints
    CONSTRAINT PK_Notifications PRIMARY KEY(NotificationID)
);

CREATE TABLE IF NOT EXISTS Customer_Services(
    MessageID VARCHAR(7),
    CustomerID VARCHAR(5),
    PhoneNumber VARCHAR(13) NOT NULL,
    Message VARCHAR(255),

    -- Constraints
    CONSTRAINT PK_Customer_Services PRIMARY KEY(MessageID),
    CONSTRAINT FK_customers_in_customer_services FOREIGN KEY (CustomerID) REFERENCES Customers(CustomerID)
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
Questions:
------
1. Is there a constraint on password in the front-end ?
2. OrderStatus get marked as Completed by the admin using the web application
3. What is the maximum speed of the robot ?
4. Does the TotalCost get calculated in the back-end of the website ?
*/