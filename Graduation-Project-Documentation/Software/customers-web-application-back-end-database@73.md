> This documentation is for issue #73, the customers web application back-end database.

<hr>

## Table of Contents ##

- [Introduction](#introduction)
- [Sign-up Page](#sign-up-page)
- [Log-in Page](#log-in-page)
- [Update Profile Page](#update-profile-page)
- [Shop and Quick View Pages](#shop-page)
- [Orders Page](#orders-page)
- [Contact Page](#contact-page)
- [Search Page](#search-page)

<hr>

## Introduction ##

Each page on the website needs a link between the customer and the system's database in order to store all the information we need as well as make the shopping process easier to the user.

<hr>

## Sign-up Page ##

When a customer needs to sign-up to our website, he enters his information and just register.

<p align="center">
<img src="https://user-images.githubusercontent.com/70551007/236686155-3e7f71ea-537e-46d6-9b0a-daefd19ad992.png">
</p>

Once he presses enter, the database stores his information.

```sql
AMR_Warehouse=# SELECT * FROM Customers;
 customerid |         email          |   password   |   fullname   | gender
------------+------------------------+--------------+--------------+--------
 C1         | mennamamdouh@gmail.com | mennamamdouh | mennamamdouh | Female
(1 row)
```

<hr>

## Log-in Page ##

I already have an account on the website and needs to log-in.

<p align="center">
<img src="https://user-images.githubusercontent.com/70551007/236686183-9a76af2f-ad09-418f-ae26-4ffaf7875cee.png">
</p>

The database searches for the email entered. If it exists in the database, the user can log-in successfully and the database direct the customer to the Home Page.

<p align="center">
<img src="https://user-images.githubusercontent.com/70551007/236686201-bbde4fc8-887a-4a23-bda4-873218ae23e1.png">
</p>

If the email doesn't exist in the __Customers__ table or the user entered wrong password, the website tells the user that he entered incorrect username or password.

<hr>

## Update Profile Page ##

If the user wants to update his password, he visits the update profile page and enters the new information.

<p align="center">
<img src="https://user-images.githubusercontent.com/70551007/236686218-bc04f926-6d07-4993-9e0e-100227ea3639.png">
</p>

After that, the new information is stored in the database.

```sql
AMR_Warehouse=# SELECT * FROM Customers;
 customerid |         email          | password  |   fullname   | gender
------------+------------------------+-----------+--------------+--------
 C1         | mennamamdouh@gmail.com | menna_new | mennamamdouh | Female
(1 row)
```

<hr>

## Shop Page ##

The shop page has all the products listed with their names, prices, and images. It retrieves this information from the database.

<p align="center">
<img src="https://user-images.githubusercontent.com/70551007/236686245-a90d94ba-8a12-41cd-89e2-5f54e370048f.png">
</p>


```sql
AMR_Warehouse=# SELECT ProductID, ProductName, Price FROM Products ORDER BY ProductID;
 productid | productname |  price
-----------+-------------+----------
 P1        | Laptop      | 13000.00
 P2        | Mouse       |  1899.00
 P3        | Camera      | 15000.00
 P4        | Mixer       |   844.00
(4 rows)
```

Each product has a _Quick View_ option to show more information about it.

<p align="center">
<img src="https://user-images.githubusercontent.com/70551007/236686307-00ceda26-b72e-4280-be75-f40310774862.png">
</p>

In this page, a customer can add a product to his wishlist, or his cart, with the quantity he needs.

1. Wishlist

   * Add a product to the Wishlist
       <p align="center">
       <img src="https://user-images.githubusercontent.com/70551007/236686342-bf3f16b4-ebaa-4c14-a9c0-afe0a302e5c2.png">
       </p>

       ```sql
       AMR_Warehouse=# SELECT * FROM Wishlist;
       customerid | productid
       ------------+-----------
       C1         | P4
       (1 row)
       ```

   * Delete a product from the Wishlist

       <p align="center">
       <img src="https://user-images.githubusercontent.com/70551007/236686369-061082ef-abca-47d0-8380-aca8a072b7a2.png">
       </p>

       ```sql
       AMR_Warehouse=# SELECT * FROM Wishlist;
       customerid | productid
       ------------+-----------
       (0 rows)
        ```

2. Cart

   * Add products to the cart

       <p align="center">
       <img src="https://user-images.githubusercontent.com/70551007/236686395-d0114d22-5767-4ce8-930f-53b5eb8f1881.png">
       </p>

       ```sql
       AMR_Warehouse=# SELECT * FROM Cart;
       customerid | productid | quantity
       ------------+-----------+----------
       C1         | P2        |        1
       C1         | P3        |        1
       (2 rows)
       ```

   * Proceed to Checkout

       <p align="center">
       <img src="https://user-images.githubusercontent.com/70551007/236686425-13597f06-dbca-484e-992c-65293975a0a7.png">
       </p>

       ```sql
       AMR_Warehouse=# SELECT * FROM Orders_Details;
       orderid | customerid | totalcost |         orderdate          | phonenumber |                        address                         |  paymentmethod   | orderstatus
       ---------+------------+-----------+----------------------------+-------------+--------------------------------------------------------+------------------+-------------
       O1      | C1         |     16899 | 2023-05-07 16:39:50.488517 | 0123456789  | flat no. 20, 16, Alexandria, Alexandria, Egypt - 12345 | Cash on Delivery | New
       (1 row)

       AMR_Warehouse=# SELECT * FROM Orders;
       orderid | productid | quantity
       ---------+-----------+----------
       O1      | P2        |        1
       O1      | P3        |        1
       (2 rows)
       ```

<hr>

## Orders Page ##

This page shows all the orders which the logged-in customer ordered.

<p align="center">
<img src="https://user-images.githubusercontent.com/70551007/236686452-bf2cb0ff-daad-462b-952d-d702fe156592.png">
</p>

This information is retrieved from the database.

* Main information about the order
    ```sql
    SELECT OD.*, C.FullName, C.Email
    FROM Orders_Details AS OD
    INNER JOIN Customers AS C
    ON C.CustomerID = OD.CustomerID
    WHERE OD.CustomerID = ?
    ```

* Information about the products
    ```sql
    SELECT STRING_AGG(CONCAT(P.ProductName, ': ', O.Quantity), ' - ') AS total_products
    FROM Orders AS O
    INNER JOIN Products AS P
        ON P.ProductID = O.ProductID
    WHERE O.OrderID = ?
    ```

<hr>

## Contact Page ##

This page lets the customer contact with our customer services. He can send any message he wants to our admins.

<p align="center">
<img src="https://user-images.githubusercontent.com/70551007/236686470-0501b38f-af94-438f-9a82-c02674990890.png">
</p>

```sql
AMR_Warehouse=# SELECT * FROM Customer_Services;
 messageid | customerid | phonenumber |        message
-----------+------------+-------------+------------------------
 M1        | C1         | 0123456789  | I love your products !
(1 row)
```

<hr>

## Search Page ##

This page lets the customer searches for any product he wants to find. The database searches for this product with its name and then displays it to the customer _if exists_.

```sql
SELECT * FROM Products WHERE LOWER(ProductName) LIKE LOWER('%{$search_box}%');
```

<p align="center">
<img src="https://user-images.githubusercontent.com/70551007/236686500-bae9304a-4490-4c0d-bd84-39f7795dbdfb.png">
</p>

<hr>
