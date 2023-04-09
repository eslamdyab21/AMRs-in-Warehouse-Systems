## <u> About the database </u> ##
Our database is a _Relational Database_ (i.e. it stores and organizes data in table consisting of rows and columns). In a relational database, data is organized into one or more tables, where each table represents a specific type of entity or object, such as customers, orders, or products.

It's built with the help of MySQL server and consists of 3 groups of tables, each group integrates with a part of our system. There's a group that integrates with the customers website, another group that integrates with the robots and shelves in the warehouse, and the last group integrates with the Admin's web application.

Mainly, the relaltionship between tables is established through a common field, known as a key, which is used to link related data across tables. The most common type of keys is _primary key_ and _foreign key_.

A _primary key_ is a column (or set of columns) that __uniquely__ identifies each record in the table, and ensures that each record has a unique value that can be used to identify and retrieve it.

A _foreign key_ is a field (or set of fields) in a table that refers to the primary key of another table. It simply establishes a relationship between two tables, where the value in the foreign key column in one table matches the value in the primary key column of another table.

It also has some _triggers_. A trigger is a database object that is associated with a table and is executed __automatically__ in response to certain database events, such as the insertion, deletion, or modification of data in the table.

<hr>

## <u> Structure of the database </u> ##

As mentioned above, the tables in our database is divided into 3 groups, each group integrates with a part of the system. Each table in any group of them has its primary key and may or may not have one or more foreign key(s).

- Tables that integrate with the website
  - __Users__
     - PK &rarr; UserID
  - __Products__
     - PK &rarr; ProductID
  - __Orders__
     - PK &rarr; OrderID
     - FKs &rarr; UserID - ProductID
		><span style="color:#73c6b6; font-weight:bold;">UserID</span> &rarr; Link _Orders_ table with _Users_ table to know which customer ordered this order.
		<br>
		><span style="color: #73c6b6; font-weight:bold;">ProductID</span> &rarr; Link _Products_ table with _Users_ table to know which product is ordered in this order.

- Tables that integrate with the robots
  - __Robots__
  	- PK &rarr; RobotID
  	- FK &rarr; ShelfID
		><span style="color:#73c6b6; font-weight:bold;">ShelfID</span> &rarr; Link _Robots_ table with _Shelves_ table to know which shelf is connected to that robot.
  - __Shelves__
  	- PK &rarr; ShelfID
  	- FK &rarr; ProductID
		><span style="color:#73c6b6; font-weight:bold;">ProductID</span> &rarr; Link _Shelves_ table with _Products_ table to know which product is on that shelf.
  
- Tables that integrate with the Admin's web application
  - __Notifications__
  	- PK &rarr; NotificationID

<hr>

## <u>Database Objects</u> ##

- Triggers  
   - __NewOrder__
     - Adds the order information in the __Notifications__ table
     - Decreases _ItemsInStock_ in the __Products__ table by _Quantity_ required by the user 
   - __CheckStates__
      - Once the _HavingOrder_ status be zero, the _ShelfID_ in the __Robots__ table be NULL
   - __CheckCost__
      - Calculates the cost of new orders (quantity x price)

<hr>

## <u> Entity Relationship Diagram (ERD) of the database </u> ##
The following Entitty Relationship Diagram (ERD) is built with the help of ERD Editor extension built-in VS Code.

<p align="center">
<img src="https://user-images.githubusercontent.com/70551007/229638460-40991392-d75f-4b93-b39e-c09313050cad.png">
</p>

It explains the information stored in each table in the database, and the relationships between all tables using their primary keys and foreign keys. It also explains the constraints defined on the data to ensure that only valid data is inserted, updated, or deleted, as well as data integrity and consistency.

<hr>

## <u> The role of the database in the system </u> ##

The database has a significant role in the flow of the system. The flow starts from the website, a customer orders an order with a specific product, then the database performs some tasks:

1. Store this order in the _Orders_ table with a new _OrderID_ and some other information such as the customer who ordered the order, the product required and its quantity, and the cost.

2. Added a new notification in the _Notifications_ table with the help of _NewOrder_ trigger. This notification should be shown on the Admin's web application.

3. Decreases the number of items in stock of this product by the quantity required by the user. It's also with the help of _NewOrder_ trigger.

4. Mark the shelf which has the required product as _HavingOrder_ and hence the algorithm will tell all robots with its ID and location. This is done because of _CheckStates_ trigger.

5. Then the algorithm does his job and finds the nearest robot to this shelf, gives it an order to move towards the shelf to move it to the packaging area.

<hr>
