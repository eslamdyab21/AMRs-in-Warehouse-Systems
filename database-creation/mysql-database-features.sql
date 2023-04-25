/*
TRIGGERS:
---------
1. NewOrder
    This trigger does 3 tasks for each new order on Orders table:
        1. Decrease the number of items in stock in Products table by the quantity of each product
        2. Mark the shelves which store the products in this order as HavingOrder shelves (isHavingOrder = 1)

2. NotifyMe
    This tigger does only one task for each new Orders_Details table:
        1. Add a notification to the Notifications table. It maps the products in this order with their shelves
        2. Calculate the total cost of the order

3. ItemsInStockUpdates
    This tigger does 2 tasks for each update on products table:
        1. If ItemsInStock < 10: Sends a notification with the product and its remaining number of items
        2. If ItemsInStock = 0: Sends a notification that the product is out of stock
*/

DELIMITER //

-- Trigger number 1 : NewOrder
CREATE TRIGGER NewOrder
AFTER INSERT ON Orders
FOR EACH ROW
BEGIN
    -- Update the number of items in stock of each product
    UPDATE Products SET ItemsInStock = ItemsInStock - NEW.Quantity WHERE ProductID = NEW.ProductID;

    -- Mark the shelves which store the products as having orders
    UPDATE Shelves SET isHavingOrder = 1 WHERE ProductID = New.ProductID;
END //

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Trigger number 2 : NotifyMe
CREATE TRIGGER NotifyMe
BEFORE INSERT ON Orders_Details
FOR EACH ROW
BEGIN
    -- Send a notification with the products and their shelves
    SET @shelves_products := (SELECT GROUP_CONCAT(CONCAT(O.Quantity, ' items of ', O.ProductID, ' from  ',  S.ShelfID)  SEPARATOR ', ') AS 'Products On Shelves'
                                FROM Orders AS O
                                INNER JOIN Shelves AS S
                                    ON O.ProductID = S.ProductID
                                WHERE O.OrderID = NEW.OrderID);
    INSERT INTO Notifications(Notification)
        VALUES(CONCAT('There is new order with ', @shelves_products, '.'));
    
    -- Calculate the total cost
    SET @total_cost := (SELECT SUM(O.Quantity * P.Price) AS TotalCost
                        FROM Orders AS O
                        INNER JOIN Products AS P
                            ON P.ProductID = O.ProductID
                        WHERE O.OrderID = NEW.OrderID);
    SET NEW.TotalCost = @total_cost;
END //

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- Trigger number 3 : ItemsInStockUpdates
CREATE TRIGGER ItemsInStockUpdates
AFTER UPDATE ON Products
FOR EACH ROW
BEGIN
    -- Check the items in stock for the updated product(s)
    DECLARE items_in_stock_notification VARCHAR(255);
    IF NEW.ItemsInStock = 0 THEN
        SET @items_in_stock_notification = (CONCAT(NEW.ProductID, ' is out of stock'));
    ELSEIF NEW.ItemsInStock < 10 THEN
        SET @items_in_stock_notification = (CONCAT(NEW.ProductID, ' is low in stock [', NEW.ItemsInStock, ' items left]'));
    END IF;

    -- Sends the notification
    IF @items_in_stock_notification IS NOT NULL THEN
        INSERT INTO Notifications(Notification)
            VALUES(@items_in_stock_notification);
    END IF;
END //


DELIMITER ;

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
To-do list:
-----------
1. Solve the problem of 'FOR EACH ROW' in adding a notification (done)
2. ItemsInStock in Products trigger (checkItemsInStock): Low or Out of Stock (done)
*/