## <u> About the database </u> ##
Our database is a _Relational Database_ (i.e. it stores and organizes data in table consisting of rows and columns). In a relational database, data is organized into one or more tables, where each table represents a specific type of entity or object, such as customers, orders, or products.

It has been built with the help of _PostgreSQL_ server and consists of 3 groups of tables, each group integrates with a part of our system. There's a group that integrates with the customers website, another group that integrates with the robots and shelves in the warehouse, and the last group integrates with the Admin's web application.

Mainly, the relaltionship between tables is established through a common field, known as a key, which is used to link related data across tables. The most common type of keys is _primary key_ and _foreign key_.

A _primary key_ is a column (or set of columns) that __uniquely__ identifies each record in the table, and ensures that each record has a unique value that can be used to identify and retrieve it.

A _foreign key_ is a field (or set of fields) in a table that refers to the primary key of another table. It simply establishes a relationship between two tables, where the value in the foreign key column in one table matches the value in the primary key column of another table.

It also has some _triggers_. A trigger is a database object that is associated with a table and is executed __automatically__ in response to certain database events, such as the insertion, deletion, or modification of data in the table.

<hr>

## <u> Structure of the database </u> ##

As mentioned above, the tables in our database is divided into 3 groups, each group integrates with a part of the system. Each table in any group of them has its primary key and may or may not have one or more foreign key(s).

- Tables that integrate with the website
  - __Customers__
     - PK &rarr; CustomerID
     - Attributes &rarr; Email, Password, Full Name, and Gender

  - __Products__
     - PK &rarr; ProductID
     - Attributes &rarr; Product Name, Price, Description, Items In Stock, and Images

  - __Orders_Details__
     - PK &rarr; OrderID
     - FKs &rarr; CustomerID
         ><span style="color:#73c6b6; font-weight:bold;">CustomeRID</span> &rarr; Link _Orders_ table with _Customers_ table to know which customer ordered this order.
     - Attributes &rarr; Total Cost, Order Date, Phone Number, Address, Payment Method, and Order Status

   - __Orders__
     - Composite PK &rarr; OrderID, ProductID
     - FKs &rarr; OrderID, ProductID
         ><span style="color: #73c6b6; font-weight:bold;">OrderID</span> &rarr; Link _Orders_ table with _Orders_Details_ table as they've the same Order IDs.

         ><span style="color: #73c6b6; font-weight:bold;">ProductID</span> &rarr; Link _Orders_ table with _Products_ table as they've the same products.
     - Attributes &rarr; Quantity

   - __Wishlist__
     - Composite PK &rarr; CustomerID, ProductID
     - FKs &rarr; CustomerID, ProductID
         ><span style="color: #73c6b6; font-weight:bold;">CustomerID</span> &rarr; Link _Wishlist_ table with _Customers_ table to know information about the customer.

         ><span style="color: #73c6b6; font-weight:bold;">ProductID</span> &rarr; Link _Wishlist_ table with _Products_ table.

   - Customer_Services
     - PK &rarr; MessageID
     - FK &rarr; CustomerID
         ><span style="color: #73c6b6; font-weight:bold;">CustomerID</span> &rarr; Link _Customer_Services_ table with _Customers_ table to who sends a message to the website.
     - Attributes &rarr; Phone Number and Message

- Tables that integrate with the robots
  - __Shelves__
  	- PK &rarr; ShelfID
  	- FK &rarr; ProductID
		  ><span style="color:#73c6b6; font-weight:bold;">ProductID</span> &rarr; Link _Shelves_ table with _Products_ table to know which product is on that shelf.
    - Attributes &rarr; Location(x, y) and Number of Orders

  - __Robots__
  	- PK &rarr; RobotID
  	- FK &rarr; ShelfID
		  ><span style="color:#73c6b6; font-weight:bold;">ShelfID</span> &rarr; Link _Robots_ table with _Shelves_ table to know which shelf is connected to that robot.
    - Attributes &rarr; Speed, Battery Percentage, Location(x, y), and Charging or not
  
- Tables that integrate with the Admin's web application
  - __Notifications__
  	- PK &rarr; NotificationID
  	- Attributes &rarr; Notification and its date and time

<hr>

## <u>Database Objects</u> ##

- Triggers  
   - __New Order__
     - Decrements the number of items in stock in Products table by the quantity of each product
     - Increment the number of orders for each shelf by 1 it the order contains the product that this shelf holds
     - Calculate the total cost of the orderrequired by the user 

   - __Items In Stock Updates__
     - If ItemsInStock < 10: Sends a notification with the product and its remaining number of items
     - If ItemsInStock = 0: Sends a notification that the product is out of stock

   - __Check Order Status__
     - Decrement the number of orders on each shelf by 1 once the order is marked as "Completed"

- Views
  - __Admin View__
    - Shows some information to the admin about the orders such as the products of each order, their quantities, their shelves, the status of the order, and the order date.

<hr>

## <u> Entity Relationship Diagram (ERD) of the database </u> ##
The following Entitty Relationship Diagram (ERD) was built with the help of [Diagrams.net](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwjEiMzopcj-AhXEYcAKHU1aAjMQFnoECAwQAQ&url=http%3A%2F%2Fdraw.io%2F&usg=AOvVaw3exHXDNauG59ZYpUDzrlsu) website.

<p align="center">
<img src="https://user-images.githubusercontent.com/70551007/229638460-40991392-d75f-4b93-b39e-c09313050cad.png">
</p>

It explains the information stored in each table in the database, and the relationships between all tables using their primary keys and foreign keys.

<hr>

## <u> The role of the database in the system </u> ##

The database has a significant role in the flow of the system. The flow starts from the customers website, a customer orders an order with specific product(s), then the database performs some tasks:

1. Stores this order in the __Orders_Details__ table with a new _OrderID_ with _OrderStatus = New_ and some other information as show in the ER Diagram. Also, stores the products in the _Orders_ table with their quantites.
    ```
    AMR_Warehouse=# SELECT * FROM Orders_Details;
    orderid | customerid | totalcost |         orderdate          |  phonenumber  |  address   |  paymentmethod   | orderstatus
    ---------+------------+-----------+----------------------------+---------------+------------+------------------+-------------
    O1      | C1         |      3350 | 2023-04-26 22:23:55.619934 | +201234567890 | Alexandria | Cash on Delivery | New
    (1 row)


    AMR_Warehouse=# SELECT * FROM Orders;         
    orderid | productid | quantity
    ---------+-----------+----------
    O1      | P1        |        5
    O1      | P2        |        7
    (2 rows)
    ```

2. Adds some information about the order, its products and the shelves which store those products to the __Admin View__. This view should be shown on the Admin's web application.
    ```
    AMR_Warehouse=# SELECT * FROM admin_view;
    orderid | productid | quantity | shelfid | orderstatus |         orderdate
    ---------+-----------+----------+---------+-------------+----------------------------
    O1      | P1        |        5 | S1      | New         | 2023-04-26 22:23:55.619934
    O1      | P2        |        7 | S2      | New         | 2023-04-26 22:23:55.619934
    (2 rows)
    ```

3. Decrements the number of items in stock of each product by the quantity required by the customer. Also, increments the number of orders for each shelf holding each product of the order's products by 1. It's done with the help of _NewOrder_ trigger.
    ```
    AMR_Warehouse=# SELECT * FROM Products;
    productid |   productname    | price  |                 description                 | itemsinstock | image_1 | image_2 | image_3
    -----------+------------------+--------+---------------------------------------------+--------------+---------+---------+---------
    P1        | Hair Shampoo     | 250.00 | It is a hair product. It contains Sulfates  |          495 | URL1    | URL2    | URL3
    P2        | Hair Conditioner | 300.00 | It is a hair product. It controls frizzness |          493 | URL1    | URL2    | URL3
    (2 rows)


    AMR_Warehouse=# SELECT * FROM Shelves;
    shelfid | location_x | location_y | productid | numoforders
    ---------+------------+------------+-----------+-------------
    S1      |          0 |          5 | P1        |           1
    S2      |          0 |         10 | P2        |           1
    (2 rows)
    ```

   * If the number of items in stock becomes less than 10 products for any of the products stored in the warehouse, a notification will be sent to the admin's web application contains that this product is _Low in Stock_.
      ```
      AMR_Warehouse=# UPDATE Products SET ItemsInStock = 7 WHERE ProductID = 'P1';
      UPDATE 1
      
      AMR_Warehouse=# SELECT * FROM Products;
      productid |   productname    | price  |                 description                 | itemsinstock | image_1 | image_2 | image_3
      -----------+------------------+--------+---------------------------------------------+--------------+---------+---------+---------
      P2        | Hair Conditioner | 300.00 | It is a hair product. It controls frizzness |          493 | URL1    | URL2    | URL3
      P1        | Hair Shampoo     | 250.00 | It is a hair product. It contains Sulfates  |            7 | URL1    | URL2    | URL3
      (2 rows)


      AMR_Warehouse=# SELECT * FROM Notifications; 
      notificationid |           notification            |    notificationdatetime
      ----------------+-----------------------------------+----------------------------
                    1 | P1 is low in stock [7 items left] | 2023-04-26 22:31:31.356656
      (1 row)
      ```
   * If the number of items in stock becomes 0 for any of the products stored in the warehouse, a notification will be sent to the admin's web application contains that this product is _Out of Stock_.
      ```
      AMR_Warehouse=# UPDATE Products SET ItemsInStock = 0 WHERE ProductID = 'P1';
      UPDATE 1
      
      AMR_Warehouse=# SELECT * FROM Products;      
      productid |   productname    | price  |                 description                 | itemsinstock | image_1 | image_2 | image_3
      -----------+------------------+--------+---------------------------------------------+--------------+---------+---------+---------
      P2        | Hair Conditioner | 300.00 | It is a hair product. It controls frizzness |          493 | URL1    | URL2    | URL3
      P1        | Hair Shampoo     | 250.00 | It is a hair product. It contains Sulfates  |            0 | URL1    | URL2    | URL3
      (2 rows)


      AMR_Warehouse=# SELECT * FROM Notifications;
      notificationid |           notification            |    notificationdatetime
      ----------------+-----------------------------------+----------------------------
                    1 | P1 is low in stock [7 items left] | 2023-04-26 22:31:31.356656
                    2 | P1 is out of stock                | 2023-04-26 22:32:48.78372
      (2 rows)
      ```

4. Then the algorithm does his job by finding the shelves which have number of orders > 0 and finds the nearest robots to those shelves.

5. Once a robot gets an order to move towards a shelf, the order associated with this shelf is marked as _In Progress_. The robot's task is to move that shelf to the packaging area.
    ```
    AMR_Warehouse=# SELECT * FROM Orders_Details;
    orderid | customerid | totalcost |         orderdate          |  phonenumber  |  address   |  paymentmethod   | orderstatus
    ---------+------------+-----------+----------------------------+---------------+------------+------------------+-------------
    O1      | C1         |      3350 | 2023-04-26 22:23:55.619934 | +201234567890 | Alexandria | Cash on Delivery | In progress
    (1 row)
    ```

6. Finally, once the admin gets all the products of that order, he marks it as _Completed_ and the number of orders of each shelf holds a product in that order is decremented by 1.
    ```
    AMR_Warehouse=# SELECT * FROM Orders_Details;
    orderid | customerid | totalcost |         orderdate          |  phonenumber  |  address   |  paymentmethod   | orderstatus
    ---------+------------+-----------+----------------------------+---------------+------------+------------------+-------------
    O1      | C1         |      3350 | 2023-04-26 22:23:55.619934 | +201234567890 | Alexandria | Cash on Delivery | Completed
    (1 row)

    AMR_Warehouse=# SELECT * FROM Shelves;  
    shelfid | location_x | location_y | productid | numoforders
    ---------+------------+------------+-----------+-------------
    S1      |          0 |          5 | P1        |           0
    S2      |          0 |         10 | P2        |           0
    (2 rows)
    ```

<hr>