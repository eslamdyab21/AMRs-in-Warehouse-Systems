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

3. CheckOrderStatus
    This tigger does only one task for each update on orders_details table:
        1. Decrement the number of orders on each shelf once the order is marked as "Completed"

VIEWS:
-----
1. AdminView
    This view views the following information to the admin:
        1. OrderID     2. ProductID     3. Quantity
        4. ShelfID     5. OrderStatus   6. OrderDate
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

-- Trigger number 3 : CheckOrderStatus
CREATE OR REPLACE FUNCTION check_order_status_function()
RETURNS TRIGGER AS $$
BEGIN
    IF NEW.OrderStatus = 'Completed' THEN
        UPDATE Shelves SET NumOfOrders = NumOfOrders - 1
            WHERE ProductID IN (SELECT ProductID FROM Orders WHERE OrderID = NEW.OrderID);
    END IF;

    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER check_order_status_trigger
AFTER UPDATE ON Orders_Details
FOR EACH ROW
EXECUTE FUNCTION check_order_status_function();

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* View for the web application */

CREATE VIEW admin_view AS
    SELECT O.OrderID, O.ProductID, O.Quantity, S.ShelfID, OD.OrderStatus, OD.OrderDate
    FROM Orders AS O
    INNER JOIN Orders_Details AS OD
        ON OD.OrderID = O.OrderID
    INNER JOIN Shelves AS S
        ON S.ProductID = O.ProductID
    ORDER BY OD.OrderDate;

-- To view it
SELECT * FROM admin_view;

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

