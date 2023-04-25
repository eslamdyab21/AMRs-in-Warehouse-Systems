/*
TRIGGERS:
---------
1. Notify Me
    This trigger does only one task for each new order:
        Add a notification to the Notifications table. It maps the products in this order with their shelves

2. New Order
    This trigger does 3 task for each new order:
        1. Decrease the number of items in stock in Products table by the quantity of each product
        2. Mark the shelves which store the products in this order as HavingOrder shelves (isHavingOrder = 1)
        3. Calculate the total cost of the order

3. ItemsInStockUpdates
    This tigger does 2 tasks for each update on products table:
        1. If ItemsInStock < 10: Sends a notification with the product and its remaining number of items
        2. If ItemsInStock = 0: Sends a notification that the product is out of stock
*/

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Trigger number 1 : NotifyMe
CREATE OR REPLACE FUNCTION notify_me_function()
RETURNS TRIGGER AS
$$
DECLARE
    shelves_products TEXT := (SELECT STRING_AGG(OD.Quantity || ' items of ' || OD.ProductID || ' from ' || S.ShelfID, ', ')
                                FROM Orders_Details AS OD
                                INNER JOIN Shelves AS S
                                    ON OD.ProductID = S.ProductID
                                WHERE OrderID = NEW.OrderID);
BEGIN
    -- Send a notification with the products and their shelves
    INSERT INTO Notifications(Notification)
    VALUES(CONCAT('There is a new order with ', shelves_products, '.'));
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER notify_me_trigger
AFTER INSERT
ON Orders_Details
FOR EACH STATEMENT
EXECUTE FUNCTION notify_me_function();

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Trigger number 2 : NewOrder
CREATE OR REPLACE FUNCTION new_order_function()
RETURNS TRIGGER AS
$$
DECLARE
    total_cost FLOAT := (SELECT SUM(OD.Quantity * P.Price) AS TotalCost
                            FROM Orders_Details AS OD
                            INNER JOIN Products AS P
                                ON P.ProductID = OD.ProductID
                            WHERE OrderID = NEW.OrderID);
BEGIN
    -- Update the number of items in stock of each product
    UPDATE Products SET ItemsInStock = ItemsInStock - NEW.Quantity WHERE ProductID = NEW.ProductID;

    -- Mark the shelves which store the products as having orders
    UPDATE Shelves SET isHavingOrder = TRUE WHERE ProductID = New.ProductID;

    -- Calculate the total cost
    UPDATE Orders SET TotalCost = @total_cost;

    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER new_order_trigger
AFTER INSERT
ON Orders_Details
FOR EACH ROW
EXECUTE FUNCTION new_order_function();

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Trigger number 3 : ItemsInStockUpdates
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
    INSERT INTO Notifications(Notification)
        VALUES(items_in_stock_notification);

    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER items_in_stock_updates_trigger
AFTER UPDATE ON Products
FOR EACH ROW
EXECUTE FUNCTION items_in_stock_updates_function();

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------