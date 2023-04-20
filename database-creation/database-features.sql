/*
TRIGGERS:
---------
1. NewOrder
    This trigger does 3 tasks for each new order:
        1. Add a notification to the Notifications table. It maps the products in this order with their shelves
        2. Decrease the number of items in stock in Products table by the quantity of each product
        3. Mark the shelves which store the products in this order as HavingOrder shelves (isHavingOrder = 1)
        4. Calculate the total cost of the order
*/

DELIMITER //
CREATE TRIGGER NewOrder
AFTER INSERT
ON Orders_Details
FOR EACH ROW
BEGIN
    -- Send a notification with the products and their shelves
    SET @shelves_products := (SELECT GROUP_CONCAT(CONCAT(OD.ProductID, ' from  ',  S.ShelfID)  SEPARATOR ', ') AS 'Products On Shelves'
                                FROM Orders_Details AS OD
                                INNER JOIN Shelves AS S
                                    ON OD.ProductID = S.ProductID
                                WHERE OrderID = NEW.OrderID);
    INSERT INTO Notifications(Notification)
        VALUES(CONCAT('There is new order with product(s) ', @shelves_products, '.'));
    
    -- Update the number of items in stock of each product
    UPDATE Products AS P SET P.ItemsInStock = P.ItemsInStock - NEW.Quantity WHERE P.ProductID = NEW.ProductID;

    -- Mark the shelves which store the products as having orders
    UPDATE Shelves AS S SET S.isHavingOrder = 1 WHERE S.ProductID = New.ProductID;

    -- Calculate the total cost
    SET @total_cost := (SELECT SUM(OD.Quantity * P.Price) AS TotalCost
                        FROM Orders_Details AS OD
                        INNER JOIN Products AS P
                            ON P.ProductID = OD.ProductID
                        WHERE OrderID = NEW.OrderID);
    UPDATE Orders AS O SET O.TotalCost = @total_cost;
END //
DELIMITER ;

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
To-do list:
-----------
1. Solve the problem of 'FOR EACH ROW' in adding a notification
*/