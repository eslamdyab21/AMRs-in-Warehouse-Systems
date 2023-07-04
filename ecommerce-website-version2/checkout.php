<?php

include 'components/connect.php';

session_start();

if(isset($_SESSION['CustomerID'])){
   $user_id = $_SESSION['CustomerID'];
}else{
   $user_id = '';
   header('location:user_login.php');
};

if(isset($_POST['order'])){

   $name = $_POST['name'];
   $name = filter_var($name, FILTER_SANITIZE_STRING);
   $number = $_POST['number'];
   $number = filter_var($number, FILTER_SANITIZE_STRING);
   $email = $_POST['email'];
   $email = filter_var($email, FILTER_SANITIZE_STRING);
   $method = $_POST['method'];
   $method = filter_var($method, FILTER_SANITIZE_STRING);
   $address = 'flat no. '. $_POST['flat'] .', '. $_POST['street'] .', '. $_POST['city'] .', '. $_POST['state'] .', '. $_POST['country'] .' - '. $_POST['pin_code'];
   $address = filter_var($address, FILTER_SANITIZE_STRING);
   $total_products = $_POST['total_products'];
   $total_price = $_POST['total_price'];

   $check_cart = $conn->prepare("SELECT * FROM Cart WHERE CustomerID = ?");
   $check_cart->execute([$user_id]);

   if($check_cart->rowCount() > 0){

      $num_of_orders = $conn->prepare("SELECT COUNT(OrderID) AS num_orders FROM Orders_Details");
      $num_of_orders->execute();
      $result = $num_of_orders->fetch(PDO::FETCH_ASSOC);
      $num_of_orders = $result["num_orders"];
      $order_id = $num_of_orders + 1;
      $order_id = 'O' . strval($order_id);

      $insert_order = $conn->prepare("INSERT INTO Orders_Details(OrderID, CustomerID, PhoneNumber, PaymentMethod, Address, TotalCost) VALUES(?,?,?,?,?,?)");
      $insert_order->execute([$order_id, $user_id, $number, $method, $address, $total_price]);

      $show_products = $conn->prepare("SELECT ProductID, Quantity FROM Cart WHERE CustomerID = ?");
      $show_products->execute([$user_id]);

      if ($show_products->rowCount() > 0) {
         while($row = $show_products->fetch(PDO::FETCH_ASSOC)){
               $product_id = $row["productid"];
               $quantity = $row["quantity"];
               $insert_product = $conn->prepare("INSERT INTO Orders(OrderID, ProductID, Quantity) VALUES(?,?,?)");
               $insert_product->execute([$order_id, $product_id, $quantity]);
            }
      }
      else {
         echo "No products!";
      }

      #$insert_products = $conn->prepare("INSERT INTO Orders(OrderID, ProductID, Quantity) VALUES(?,?,?)");
      #$insert_products->execute([$next_id, $productid, $quantity]);

      $delete_cart = $conn->prepare("DELETE FROM Cart WHERE CustomerID = ?");
      $delete_cart->execute([$user_id]);

      $message[] = 'order placed successfully!';
   }else{
      $message[] = 'your cart is empty';
   }

}

?>

<!DOCTYPE html>
<html lang="en">
<head>
   <meta charset="UTF-8">
   <meta http-equiv="X-UA-Compatible" content="IE=edge">
   <meta name="viewport" content="width=device-width, initial-scale=1.0">
   <title>checkout</title>
   
   <!-- font awesome cdn link  -->
   <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.1.1/css/all.min.css">

   <!-- custom css file link  -->
   <link rel="stylesheet" href="css/style.css">

</head>
<body>
   
<?php include 'components/user_header.php'; ?>

<section class="checkout-orders">

   <form action="" method="POST">

   <h3>your orders</h3>

      <div class="display-orders">
      <?php
         $grand_total = 0;
         $cart_items[] = '';
         $select_cart = $conn->prepare("SELECT *, P.Price, P.ProductName FROM Cart AS C
                                          INNER JOIN Products AS P
                                             ON P.ProductID = C.ProductID
                                          WHERE C.CustomerID = ?");
         $select_cart->execute([$user_id]);
         if($select_cart->rowCount() > 0){
            while($fetch_cart = $select_cart->fetch(PDO::FETCH_ASSOC)){
               $cart_items[] = $fetch_cart['productname'].' ('.$fetch_cart['price'].' x '. $fetch_cart['quantity'].') - ';
               $total_products = implode($cart_items);
               $grand_total += ($fetch_cart['price'] * $fetch_cart['quantity']);
      ?>
         <p> <?= $fetch_cart['productname']; ?> <span>(<?= '$'.$fetch_cart['price'].'/- x '. $fetch_cart['quantity']; ?>)</span> </p>
      <?php
            }
         }else{
            echo '<p class="empty">your cart is empty!</p>';
         }
      ?>
         <input type="hidden" name="total_products" value="<?= $total_products; ?>">
         <input type="hidden" name="total_price" value="<?= $grand_total; ?>" value="">
         <div class="grand-total">grand total : <span>$<?= $grand_total; ?>/-</span></div>
      </div>

      <h3>place your orders</h3>

      <div class="flex">
         <div class="inputBox">
            <span>your name :</span>
            <input type="text" name="name" placeholder="enter your name" class="box" maxlength="20" required>
         </div>
         <div class="inputBox">
            <span>your number :</span>
            <input type="number" name="number" placeholder="enter your number" class="box" min="0" max="9999999999" onkeypress="if(this.value.length == 10) return false;" required>
         </div>
         <div class="inputBox">
            <span>your email :</span>
            <input type="email" name="email" placeholder="enter your email" class="box" maxlength="50" required>
         </div>
         <div class="inputBox">
            <span>payment method :</span>
            <select name="method" class="box" required>
               <option value="Cash on Delivery">Cash on Delivery</option>
               <option value="Credit Card">Credit Card</option>
               <option value="Paytm">Paytm</option>
               <option value="Paypal">Paypal</option>
            </select>
         </div>
         <div class="inputBox">
            <span>address line 01 :</span>
            <input type="text" name="flat" placeholder="e.g. flat number" class="box" maxlength="50" required>
         </div>
         <div class="inputBox">
            <span>address line 02 :</span>
            <input type="text" name="street" placeholder="e.g. street name" class="box" maxlength="50" required>
         </div>
         <div class="inputBox">
            <span>city :</span>
            <input type="text" name="city" placeholder="e.g. mumbai" class="box" maxlength="50" required>
         </div>
         <div class="inputBox">
            <span>state :</span>
            <input type="text" name="state" placeholder="e.g. maharashtra" class="box" maxlength="50" required>
         </div>
         <div class="inputBox">
            <span>country :</span>
            <input type="text" name="country" placeholder="e.g. India" class="box" maxlength="50" required>
         </div>
         <div class="inputBox">
            <span>pin code :</span>
            <input type="number" min="0" name="pin_code" placeholder="e.g. 123456" min="0" max="999999" onkeypress="if(this.value.length == 6) return false;" class="box" required>
         </div>
      </div>

      <input type="submit" name="order" class="btn <?= ($grand_total > 1)?'':'disabled'; ?>" value="place order">

   </form>

</section>













<?php include 'components/footer.php'; ?>

<script src="js/script.js"></script>

</body>
</html>