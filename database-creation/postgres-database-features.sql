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