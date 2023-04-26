/*
- To use Postgres here: psql -h host_name -p port_number -U user_name -d postgres
- To show statistics of time: EXPLAIN ANALYZE query
- To disconnet all users from a database in order to drop it :
      SELECT pg_terminate_backend(pg_stat_activity.pid)
      FROM pg_stat_activity
      WHERE pg_stat_activity.datname = 'database_name';
- To connect to a database: \c database_name
- To drop a database: DROP DATABASE database_name;
- To show tables in the database: \dt
- To reset counting on auto-incremental columns: ALTER SEQUENCE my_table_id_seq RESTART WITH 1;
*/

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

CREATE DATABASE "AMR_Warehouse";

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* Interacting with the website */

CREATE TABLE IF NOT EXISTS Customers(
    CustomerID VARCHAR(7),
    Email VARCHAR(50),
    Password VARCHAR(20),
    FullName VARCHAR(25),
    Gender VARCHAR(6) CHECK(Gender IN ('Male', 'Female')),

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

CREATE TABLE IF NOT EXISTS Orders_Details(
    OrderID VARCHAR(7),
    CustomerID VARCHAR(7),
    TotalCost FLOAT CHECK(TotalCost >= 0) DEFAULT NULL,
    OrderDate TIMESTAMP DEFAULT NOW(),
    PhoneNumber VARCHAR(13),
    Address VARCHAR(255),
    PaymentMethod VARCHAR(20) CHECK(PaymentMethod IN ('Cash on Delivery', 'Credit Card', 'Paypal', 'Paytm')),
    OrderStatus VARCHAR(15) DEFAULT 'New' CHECK(OrderStatus IN ('New', 'In progress', 'Completed')),

    -- Constraints
    CONSTRAINT PK_Orders_Details PRIMARY KEY(OrderID),
    CONSTRAINT FK_customers_in_orders_details FOREIGN KEY (CustomerID) REFERENCES Customers(CustomerID)
);

CREATE TABLE IF NOT EXISTS Orders(
    OrderID VARCHAR(7),
    ProductID VARCHAR(7),
    Quantity INT CHECK(Quantity > 0),

    -- Constraints
    CONSTRAINT PK_Orders PRIMARY KEY(OrderID, ProductID),
    CONSTRAINT FK_orders_in_orders_details FOREIGN KEY (OrderID) REFERENCES Orders_Details(OrderID),
    CONSTRAINT FK_products_in_orders FOREIGN KEY (ProductID) REFERENCES Products(ProductID)
);

CREATE TABLE IF NOT EXISTS Wishlist(
    CustomerID VARCHAR(7),
    ProductID VARCHAR(7),

    -- Constraints
    CONSTRAINT PK_Wishlist PRIMARY KEY(CustomerID, ProductID),
    CONSTRAINT FK_customers_in_wishlist FOREIGN KEY (CustomerID) REFERENCES Customers(CustomerID),
    CONSTRAINT FK_products_in_wishlist FOREIGN KEY (ProductID) REFERENCES Products(ProductID)
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

/* Interacting with the warehouse */

CREATE TABLE IF NOT EXISTS Shelves(
    ShelfID VARCHAR(7),
    Location_X INT NOT NULL CHECK(Location_X BETWEEN 0 AND 20),
    Location_Y INT NOT NULL CHECK(Location_Y BETWEEN 0 AND 20),
    ProductID VARCHAR(7), -- The product that the shelf stores
    NumOfOrders INT DEFAULT 0 CHECK(NumOfOrders >= 0),

    -- Constraints
    CONSTRAINT PK_Shelves PRIMARY KEY(ShelfID),
    CONSTRAINT FK_products_in_shelves FOREIGN KEY (ProductID) REFERENCES Products(ProductID),
    CONSTRAINT shelves_unique_locations UNIQUE (Location_X, Location_Y)
);

CREATE TABLE IF NOT EXISTS Robots(
    RobotID VARCHAR(7),
    Speed INT CHECK(Speed BETWEEN 0 AND 100),
    BatteryPercentage INT NOT NULL CHECK(BatteryPercentage BETWEEN 0 AND 100),
    CurrentLocation_X INT NOT NULL CHECK(CurrentLocation_X BETWEEN 0 AND 20),
    CurrentLocation_Y INT NOT NULL CHECK(CurrentLocation_Y BETWEEN 0 AND 20),
    isCharging BOOL,
    ShelfID VARCHAR(7), -- The shelf that the robot holds

    -- Constraints
    CONSTRAINT PK_Robotos PRIMARY KEY(RobotID),
    CONSTRAINT FK_shelves_with_robots FOREIGN KEY (ShelfID) REFERENCES Shelves(ShelfID),
    CONSTRAINT robots_unique_locations UNIQUE (CurrentLocation_X, CurrentLocation_Y),
    CONSTRAINT unique_shelves UNIQUE(ShelfID)
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* For the admin */

CREATE TABLE IF NOT EXISTS Notifications(
    NotificationID SERIAL,
    Notification VARCHAR(255),
    NotificationDateTime TIMESTAMP DEFAULT NOW(),

    -- Constraints
    CONSTRAINT PK_Notifications PRIMARY KEY(NotificationID)
);

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------