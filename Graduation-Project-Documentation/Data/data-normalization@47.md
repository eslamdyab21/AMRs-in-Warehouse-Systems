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
   <img src="https://user-images.githubusercontent.com/70551007/234450273-1faee369-d9ef-462e-95d1-4f47fdc3411d.png">
   </p>

   __Customers__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF as well as there's no _transitive dependency_.

<br>

2. __Orders__:
 
   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/234450319-2302613a-398f-44af-9570-0a1d1f4cfcbf.png">
   </p>

   We noticed that this table has two multi-valued attributes, _ProductID_ and _Quantity_, as the customer may order more than one product.
   So, this table is not in 1NF. To make it in 1NF, we'll move the _ProductID_ and _Quantity_ in another table with a composite primary key (OrderID, ProductID).

   Now the __Orders__ table is divided into 2 tables:
   1. __Orders_Details__: The original orders table without _ProductID_, and _Quantity_
   2. __Orders__

   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/234450356-17b4aed3-2558-4bb6-87e4-283dd1ec1f95.png">
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
   <img src="https://user-images.githubusercontent.com/70551007/234450409-ca3b657c-9447-4225-b878-018671541217.png">
   </p>

   __Products__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

4. __Robots__:
 
   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/234450447-e1153df5-4f29-4c5d-91de-08b537c5c5fc.png">
   </p>

   __Robots__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

5. __Shelves__:

   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/234450489-b99f3c0c-1764-451a-bb33-cc9822cece83.png">
   </p>

   __Shelves__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

6. __Notifications__:
 
   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/234450584-90067926-ca22-46cb-965a-d12513139559.png">
   </p>

   __Notifications__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

7. __Customer_Services__:
 
   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/234450653-61a7d1e2-eed0-4d4c-9f82-f20c3c4208a4.png">
   </p>

   __Customer_Services__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

8. __Wishlist__:

   <p align="center">
   <img src="https://user-images.githubusercontent.com/70551007/234450697-1415f449-4d44-46ac-bb65-2e6275e5d572.png">
   </p>

   __Wishlist__ is in 1NF as it doesn't have any multi-valued attributes, and in 2NF as all the non-key attributes are fully dependent on the primary key. It is in 3NF too as there's no _transitive dependency_.

<br>

Now, the database normalization process is done :heavy_check_mark:.
