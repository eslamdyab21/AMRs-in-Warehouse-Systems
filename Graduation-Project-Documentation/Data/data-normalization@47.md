This is a documentation for issue #47, Database Normalization.

<hr>

## About Data Normalization ##

Database Normalization is a process of organizing the data in a relational database so that it is strucutured in a way that reduces __redundancy__ and __dependency__. The main goal of normalization is to eliminate __data redundancy__ and ensure that each piece of data is stored in only one place, thereby reducing the likelihood of data inconsistencies.

Normalization is typically achieved through a series of steps known as _normal forms_, with each normal form building on the previous one. There are several normal forms, including:
1. __First Normal Form (1NF)__: This requires that each table has a primary key and that all columns in the table are _atomic_, meaning that they cannot be further subdivided or have many values.
2. __Second Normal Form (2NF)__: This requires that all non-key attributes in a table are fully dependent on the primary key, meaning that there's no _partical dependency_ if there's more than one primary key.
3. __Third Normal Form (3NF)__: This requires that all non-key attributes in a table are independent of each other, meaning that there is no _transitive dependency_ between non-key attributes.

There're additional normal forms beyond __3NF__, including __Boyce-Codd Normal Form (BCNF)__ and __Fourth Normal Form (4NF)__, which further refine the normalization process.

<hr>

## Applying data normalization on our database ##

We'll apply the data normalization process on each table in our database to ensure that there's no __data redundancy__, __partial__, or __transitive dependency__.

1. __Customers__:

   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232264280-51adee4c-80d2-4f9c-9202-cb971ccdd936.png">
   </p>

   __Customers__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF as well as there's no _transitive dependency_.

<br>

2. __Orders__:
   
   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232264285-3d3fce2b-0efd-4daa-b1e1-60b749b6e611.png">
   </p>

   We noticed that this table has two multi-valued attributes, _ProductID_ and _Quantity_, as the customer may order more than one product.
   So, this table is not in 1NF. To make it in 1NF, we'll move the _ProductID_ and _Quantity_ in another table with a composite primary key (OrderID, ProductID).

   Now the __Orders__ table is divided into 2 tables:
   1. __Orders_Details__: The original orders table without _ProductID_, and _Quantity_
   2. __Orders__

   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232264325-762a2b76-c870-4ae8-95f6-8894a2447a69.png">
   </p>

   Now, we've to examine both tables:
   1. __Orders_Details__:
      - There's no _partial dependency_ as we've only one primary key on which all the attributes depend. So, we can say that the table is in 2NF now.         
      - There's no _transitive dependency_ as there's no non-key attribute which depends on another non-key attribute. So, the table is in 3NF.
   2. __Orders__:
      - There's no _partial dependency_ as _Quantity_ depends on both (OrderID, ProductID). So, the table is in 2NF.
      - There's no _transitive dependency_, so the table is in 3NF.

<br>

3. __Products__:
      
   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232264334-3856d203-6906-49b0-aeb6-86d9cea07144.png">
   </p>

   __Products__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

4. __Robots__:
   
   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232264344-f4af69dc-207d-401e-ac65-dc2cfc74c08e.png">
   </p>

   __Robots__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

5. __Shelves__:

   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232264353-0f11a194-8421-4af3-8417-a7b6104160dd.png">
   </p>

   __Shelves__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

6. __Notifications__:
   
   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232258545-e6bfe7aa-89ef-4636-953b-71954eeb5ff9.png">
   </p>

   __Notifications__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

7. __Customer_Services__:
   
   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232264367-825a6baa-9c3e-4231-9727-723ba9c8de8d.png">
   </p>

   __Customer_Services__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

8. __Wishlist__:

   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232264367-825a6baa-9c3e-4231-9727-723ba9c8de8d.png">
   </p>

   __Wishlist__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

Now, the database normalization process is done :heavy_check_mark:.