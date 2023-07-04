<?php

include 'components/connect.php';

session_start();

if(isset($_SESSION['CustomerID'])){
   $user_id = $_SESSION['CustomerID'];
}else{
   $user_id = '';
};

?>

<!DOCTYPE html>
<html lang="en">
<head>
   <meta charset="UTF-8">
   <meta http-equiv="X-UA-Compatible" content="IE=edge">
   <meta name="viewport" content="width=device-width, initial-scale=1.0">
   <title>orders</title>
   
   <!-- font awesome cdn link  -->
   <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.1.1/css/all.min.css">

   <!-- custom css file link  -->
   <link rel="stylesheet" href="css/style.css">

</head>
<body>
   
<?php include 'components/user_header.php'; ?>

<section class="orders">

   <h1 class="heading">placed orders</h1>

   <div class="box-container">

   <?php
      if($user_id == ''){
         echo '<p class="empty">please login to see your orders</p>';
      }else{
         $select_orders = $conn->prepare("SELECT OD.*, C.FullName, C.Email
                                             FROM Orders_Details AS OD
                                             INNER JOIN Customers AS C
                                                ON C.CustomerID = OD.CustomerID
                                             WHERE OD.CustomerID = ?");
         $select_orders->execute([$user_id]);

         if($select_orders->rowCount() > 0){
            while($fetch_orders = $select_orders->fetch(PDO::FETCH_ASSOC)){

               $fetch_products = $conn->prepare("SELECT STRING_AGG(CONCAT(P.ProductName, ': ', O.Quantity), ' - ') AS total_products
                                                   FROM Orders AS O
                                                   INNER JOIN Products AS P
                                                      ON P.ProductID = O.ProductID
                                                   WHERE O.OrderID = ?");
               $fetch_products->execute([$fetch_orders['orderid']]);
               $fetch_products = $fetch_products->fetch(PDO::FETCH_ASSOC);

   ?>
   <div class="box">
      <p>placed on : <span><?= $fetch_orders['orderdate']; ?></span></p>
      <p>name : <span><?= $fetch_orders['fullname']; ?></span></p>
      <p>email : <span><?= $fetch_orders['email']; ?></span></p>
      <p>number : <span><?= $fetch_orders['phonenumber']; ?></span></p>
      <p>address : <span><?= $fetch_orders['address']; ?></span></p>
      <p>payment method : <span><?= $fetch_orders['paymentmethod']; ?></span></p>
      <p>your orders : <span><?= $fetch_products['total_products']; ?></span></p>
      <p>total price : <span>$<?= $fetch_orders['totalcost']; ?>/-</span></p>
   </div>
   <?php
      }
      }else{
         echo '<p class="empty">no orders placed yet!</p>';
      }
      }
   ?>

   </div>

</section>













<?php include 'components/footer.php'; ?>

<script src="js/script.js"></script>

</body>
</html>