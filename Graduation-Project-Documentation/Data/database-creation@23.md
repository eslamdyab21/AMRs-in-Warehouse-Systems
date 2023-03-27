## <u> About the database </u> ##
Our database is built on MySQL server. It consists of multiple groups of tables, each group of them integrates with a part of our system.

There're some tables for integrating with the website, some for integrating with the robots in the warehouse, and some for integrating with the Admin's web application.

<hr>

## <u> Structure of the database </u> ##

1. __Tables that integrate with the website__
	- Users
    	- UserID --> (PK)
    	- Email
    	- Paddword
    	- Fullame
    	- Gender
	- Products
    	- ProductID --> (PK)
    	- Price
    	- ItemsInStock
	- Orders
    	- OrderID --> (PK)
    	- UserID --> (FK)
    	- ProductID --> (FK)
    	- Quantity
    	- Cost
    	- Date
  
2. __Tables that integrate with the robots__
	- Robots
    	- RobotID --> (PK)
    	- Speed
    	- BatteryLife
    	- CurrentLocation
    	- NextLocation
    	- ShelfID --> (FK)
    	- HavingOrder
    	- Moving
	- Shelves
    	- ShelfID --> (PK)
    	- Location
    	- ProductID --> (FK)
    	- HavingOrder
  
3. __Tables that integrate with the Admin's web application__
	- Notifications
    	- NotificationID
    	- Notification
    	- Date

4. __Triggers__
   - <u>NewOrder:</u>
     - Adds the order information in the __Notifications__ table
  
     - Decreases _ItemsInStock_ in the __Products__ table by _Quantity_ required by the user
  
   - <u>CheckStates:</u>
     - Once the _HavingOrder_ status be zero, the _ShelfID_ in the __Robots__ table be NULL

   - <u>CheckCost:</u>
     - Calculates the cost of new orders (quantity x price)

<hr>

## <u> The role of the database in the system </u> ##

The database has a significant role in the flow of the system. The flow starts from the website, a customer orders an order with a specific product, then the database performs some tasks:

1. Store this order in the __Orders__ table with a new _OrderID_ and the other information about the user, the product, the quantity required, the cost, and the order date.

2. Added a new notification in the __Notifications__ table with the product

3. Decreases the number of items in stock of this product by the quantity required by the user.

4. Mark the shelf which has the required product as _HavingOrder_ and tell all robots with its ID

5. Then the algorithm does his job and finds the nearest robot to this shelf, moves it, and gets the shelf to the packaging area

<hr>

## <u> Entity Relationship Diagram (ERD) of the database </u> ##
The following Entitty Relationship Diagram (ERD) is built with the help of ERD Editor extension build in VS Code.

<img src="images\ERD.png">

It explains the information stored in each table in the database, and the relationships between all tables with their primary keys and foreign keys.

<hr>