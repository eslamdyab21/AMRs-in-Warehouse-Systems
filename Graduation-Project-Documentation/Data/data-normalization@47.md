This is a documentation for issue #47, the Database Normalization.

<hr>

## About Data Normalization ##

Database Normalization is a process of organizing the data in a relational database so that it is strucutured in a way that reduces __redundancy__ and __dependency__. The main goal of normalization is to eliminate __data redundancy__ and ensure that each piece of data is stored in only one place, thereby reducing the likelihood of data inconsistencies.

Normalization is typically achieved through a series of steps known as _normal forms_, with each normal form building on the previous one.

There are several normal forms, including:
1. **First Normal Form (1NF)**: This requires that each table has a primary key and that all columns in the table are _atomic_, meaning that they cannot be further subdivided or have many values.
2. **Second Normal Form (2NF)**: This requires that all non-key attributes in a table are fully dependent on the primary key, meaning that there's no _partical dependency_ if there's more than one primary key.
3. **Third Normal Form (3NF)**: This requires that all non-key attributes in a table are independent of each other, meaning that there is no _transitive dependency_ between non-key attributes.

There're additional normal forms beyond 3NF, including __Boyce-Codd Normal Form (BCNF)__ and __Fourth Normal Form (4NF)__, which further refine the normalization process.

<hr>

## Applying data normalization on our database ##

We'll apply the data normalization process on each table in our database to ensure that there's no __data redundancy__ and __partial and transitive dependency__.

1. __Customers__:

   <p align="center">
   <img src="Customers_Table">
   </p>

   __Customers__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF as well as there's no _transitive dependency_.

<br>

2. __Orders__:
   
   <p align="center">
   <img src="Orders_Table">
   </p>

   We noticed that this table has two multi-valued attributes, _ProductID_ and _Quantity_, as the customer may order more than one product.
   So, this table is not in 1NF. To make it in 1NF, we'll move the _ProductID_ and _Quantity_ in another table with a composite primary key (OrderID, ProductID).

   Now the __Orders__ table is divided into 2 tables:
   1. __Orders__: The default orders table without _ProductID_, and _Quantity
   2. __Orders_Details__

   <p align="center">
   <img src="Orders_1NF">

   Now, we've to examine both tables:
   1. __Orders__: There's no _partial dependency_ as we've only one primary key, and all attributes depend on it. So, we can say that the table is in 2NF. There's no _transitive dependency_ too as there's no non-key attribute which depends on another non-key attribute. So, the table is in 3NF.
   2. __Orders_Details__: There's no _partial dependency_ as _Quantity_ depends on both (OrderID, ProductID). So, the table is in 2NF. There's no _transitive dependency_, so the table is in 3NF.

<br>

3. __Products__:
   
   <p align="center">
   <img src="Products_Table">
   </p>

   __Products__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

4. __Robots__:
   
   <p align="center">
   <img src="Robots_Table">
   </p>

   __Robots__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

5. __Shelves__:
   
   <p align="center">
   <img src="Shelves_Table">
   </p>

   __Shelves__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

6. __Notifications__:
   
   <p align="center">
   <img src="Notifications_Table">
   </p>

   __Notifications__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

7. __Customer_Services__:
   
   <p align="center">
   <img src="Customer_Services_Table">
   </p>

   __Customer_Services__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

Now, the database normalization process is done :heavy_check_mark:.