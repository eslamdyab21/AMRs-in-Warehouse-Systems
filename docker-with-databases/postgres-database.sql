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
    OrderProductStatus VARCHAR(15) DEFAULT 'New' CHECK(OrderProductStatus IN ('New', 'In progress', 'Completed')),

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

CREATE TABLE Cart(
    CustomerID VARCHAR(7),
    ProductID VARCHAR(7),
    Quantity INT CHECK(Quantity > 0),

    -- Constraints
    CONSTRAINT PK_Cart PRIMARY KEY(CustomerID, ProductID),
    CONSTRAINT FK_customers_in_cart FOREIGN KEY (CustomerID) REFERENCES Customers(CustomerID),
    CONSTRAINT FK_products_in_cart FOREIGN KEY (ProductID) REFERENCES Products(ProductID)
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

/*
TRIGGERS:
---------
1. NewOrder
    This trigger does 3 task for each new order:
        1. Decrease the number of items in stock in Products table by the quantity of each product
        2. Increment the number of orders for each shelf by 1 it the order contains the product that this shelf holds
        3. Calculate the total cost of the order

2. ItemsInStockUpdates
    This tigger does 2 tasks for each update on products table:
        1. If ItemsInStock < 10: Sends a notification with the product and its remaining number of items
        2. If ItemsInStock = 0: Sends a notification that the product is out of stock

3. ShelfPairedWithRobot
    This trigger does only one task for each update on robots table:
        When a robot is paired with a shelf (the database knows this information from the algorithm),
        it updates each orderid and productid (on the paired shelf) pair in the orders table as in progress instead of new.

4. CompletedOrders
    This trigger does 2 tasks for each update on order tables:
        1.  Decrement the number of orders on each shelf (according to its product) once the order and product pair is marked as "Completed"
        2. If all the products of any order is marked as completed, it updates the order status of this order in the orders_details table as "Completed" 


VIEWS:
-----
1. AdminView
    This view views the following information to the admin:
        1. OrderID     2. ProductID     3. Quantity
        4. ShelfID     5. OrderProductStatus   6. OrderDate
    To monitor the orders and the shelves.
*/

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Trigger number 1 : NewOrder
CREATE OR REPLACE FUNCTION new_order_function()
RETURNS TRIGGER AS
$$
DECLARE
    total_cost FLOAT := (SELECT SUM(O.Quantity * P.Price) AS TotalCost
                            FROM Orders AS O
                            INNER JOIN Products AS P
                                ON P.ProductID = O.ProductID
                            WHERE OrderID = NEW.OrderID);
BEGIN
    -- Update the number of items in stock of each product
    UPDATE Products SET ItemsInStock = ItemsInStock - NEW.Quantity WHERE ProductID = NEW.ProductID;

    -- Update the number of orders in shelves
    UPDATE Shelves SET NumOfOrders = NumOfOrders + 1 WHERE ProductID = NEW.ProductID;

    -- Calculate the total cost
    UPDATE Orders_Details SET TotalCost = @total_cost;

    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER new_order_trigger
AFTER INSERT
ON Orders
FOR EACH ROW
EXECUTE FUNCTION new_order_function();

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Trigger number 2 : ItemsInStockUpdates
CREATE OR REPLACE FUNCTION items_in_stock_updates_function()
RETURNS TRIGGER AS $$
DECLARE
    items_in_stock_notification VARCHAR(255);
BEGIN
    -- Check the items in stock for the updated product(s)
    IF NEW.ItemsInStock = 0 THEN
        items_in_stock_notification = CONCAT(NEW.ProductID, ' is out of stock');
    ELSEIF NEW.ItemsInStock < 10 THEN
        items_in_stock_notification = CONCAT(NEW.ProductID, ' is low in stock [', NEW.ItemsInStock, ' items left]');
    END IF;

    -- Sends the notification
    IF items_in_stock_notification IS NOT NULL THEN
        INSERT INTO Notifications(Notification)
            VALUES(items_in_stock_notification);
    END IF;

    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER items_in_stock_updates_trigger
AFTER UPDATE ON Products
FOR EACH ROW
EXECUTE FUNCTION items_in_stock_updates_function();

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Trigger number 3 : ShelfPairedWithRobot
CREATE OR REPLACE FUNCTION shelf_paired_with_robot_function()
RETURNS TRIGGER AS $$
DECLARE
    product_id VARCHAR(7) := (SELECT ProductID FROM Shelves WHERE ShelfID = NEW.ShelfID);
BEGIN
    IF NEW.ShelfID IS NOT NULL THEN
        UPDATE Orders SET OrderProductStatus = 'In progress'
            WHERE OrderID IN (SELECT OrderID FROM Orders WHERE ProductID = product_id) AND ProductID = product_id;
    END IF;

    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER shelf_paired_with_robot_trigger
AFTER UPDATE OF ShelfID ON Robots
FOR EACH ROW
EXECUTE FUNCTION shelf_paired_with_robot_function();

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Trigger number 4 : CompletedOrders
CREATE OR REPLACE FUNCTION completed_order_function()
RETURNS TRIGGER AS
$$
DECLARE
    order_id VARCHAR(7) := NEW.OrderID;

    total_products INT := (SELECT COUNT(OrderID) FROM Orders WHERE OrderID = order_id);
    total_completed_products INT := (SELECT COUNT(OrderID) FROM Orders WHERE OrderID = order_id AND OrderProductStatus = 'Completed');

    product_id VARCHAR(7) := NEW.ProductID;
    shelf_id VARCHAR(7) := (SELECT ShelfID FROM Shelves WHERE ProductID = product_id);

BEGIN
    UPDATE Shelves SET NumOfOrders = NumOfOrders - 1 WHERE ShelfID = shelf_id;

    IF total_completed_products = total_products THEN
        UPDATE Orders_Details SET OrderStatus = 'Completed' WHERE OrderID = order_id;
    END IF;

    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER completed_order_trigger
AFTER UPDATE
ON Orders
FOR EACH ROW
EXECUTE FUNCTION completed_order_function();

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* View for the web application */

CREATE VIEW admin_view AS
    SELECT O.OrderID, O.ProductID, O.Quantity, S.ShelfID, O.OrderProductStatus, OD.OrderDate
    FROM Orders AS O
    INNER JOIN Orders_Details AS OD
        ON OD.OrderID = O.OrderID
    INNER JOIN Shelves AS S
        ON S.ProductID = O.ProductID
    ORDER BY OD.OrderDate;

-- To view it
SELECT * FROM admin_view;