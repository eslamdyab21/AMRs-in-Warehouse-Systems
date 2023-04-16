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
   <img src="https://user-images.githubusercontent.com/70551007/232258468-099d14aa-770d-4e0d-a381-d95b12f2aa9c.png">
   </p>

   __Customers__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF as well as there's no _transitive dependency_.

<br>

2. __Orders__:
   
   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232259100-b242eafb-d1bd-41bf-aca0-0d30dd62e3ef.png">
   </p>

   We noticed that this table has two multi-valued attributes, _ProductID_ and _Quantity_, as the customer may order more than one product.
   So, this table is not in 1NF. To make it in 1NF, we'll move the _ProductID_ and _Quantity_ in another table with a composite primary key (OrderID, ProductID).

   Now the __Orders__ table is divided into 2 tables:
   1. __Orders__: The original orders table without _ProductID_, and _Quantity_
   2. __Orders_Details__

   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232259072-1ff25b57-111f-4211-bc96-dd5a0a77e8d6.png">
   </p>

   Now, we've to examine both tables:
   1. __Orders__:
      - There's no _partial dependency_ as we've only one primary key on which all the attributes depend. So, we can say that the table is in 2NF now.         
      - There's no _transitive dependency_ as there's no non-key attribute which depends on another non-key attribute. So, the table is in 3NF.
   2. __Orders_Details__:
      - There's no _partial dependency_ as _Quantity_ depends on both (OrderID, ProductID). So, the table is in 2NF.
      - There's no _transitive dependency_, so the table is in 3NF.

<br>

3. __Products__:
      
   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232258519-eaefd40d-e380-430c-a4de-5b7c3fdc7369.png">
   </p>

   __Products__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

4. __Robots__:
   
   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232258532-dbf38007-6b85-44a8-bfde-526cabb22286.png">
   </p>

   __Robots__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

5. __Shelves__:

   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/232258537-6a342988-497f-4d46-9ca2-54c3a62afc3f.png">
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
   <img src="https://user-images.githubusercontent.com/70551007/232258555-e6788a51-ef3f-4e41-9016-240eda2bbe77.png">
   </p>

   __Customer_Services__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

Now, the database normalization process is done :heavy_check_mark:.
