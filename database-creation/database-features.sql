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
    SET @shelves_products := (SELECT GROUP_CONCAT(CONCAT(O.ProductID, ' from  ',  S.ShelfID)  SEPARATOR ', ') AS 'Products On Shelves'
                                FROM Orders AS O
                                INNER JOIN Shelves AS S
                                    ON O.ProductID = S.ProductID
                                WHERE O.OrderID = NEW.OrderID);
    INSERT INTO Notifications(Notification)
        VALUES(CONCAT('There is new order with product(s) ', @shelves_products, '.'));
    
    -- Calculate the total cost
    SET @total_cost := (SELECT SUM(O.Quantity * P.Price) AS TotalCost
                        FROM Orders AS O
                        INNER JOIN Products AS P
                            ON P.ProductID = O.ProductID
                        WHERE O.OrderID = NEW.OrderID);
    SET NEW.TotalCost = @total_cost;
END //
DELIMITER ;

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
To-do list:
-----------
1. Solve the problem of 'FOR EACH ROW' in adding a notification (done)
2. ItemsInStock in Products trigger (checkItemsInStock): Low or Out of Stock
*/