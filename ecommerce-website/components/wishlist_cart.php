<?php

if(isset($_POST['add_to_wishlist'])){
   if($user_id == ''){
      header('location:user_login.php');
   }else{

      $pid = $_POST['pid'];
      $pid = filter_var($pid, FILTER_SANITIZE_STRING);
      $name = $_POST['name'];
      $name = filter_var($name, FILTER_SANITIZE_STRING);
      $price = $_POST['price'];
      $price = filter_var($price, FILTER_SANITIZE_STRING);
      $image = $_POST['image'];
      $image = filter_var($image, FILTER_SANITIZE_STRING);

      $check_wishlist_numbers = $conn->prepare("SELECT * FROM Wishlist AS W
                                                INNER JOIN Products AS P
                                                   ON P.ProductID = W.ProductID
                                                WHERE P.ProductName = ? AND W.CustomerID = ?");
      $check_wishlist_numbers->execute([$name, $user_id]);

      $check_cart_numbers = $conn->prepare("SELECT * FROM Cart AS C
                                             INNER JOIN Products AS P
                                                ON P.ProductID = C.ProductID
                                             WHERE P.ProductName = ? AND C.CustomerID = ?");
      $check_cart_numbers->execute([$name, $user_id]);

      if($check_wishlist_numbers->rowCount() > 0){
         $message[] = 'Already added to wishlist!';
      }elseif($check_cart_numbers->rowCount() > 0){
         $message[] = 'Already added to cart!';
      }else{
         $insert_wishlist = $conn->prepare("INSERT INTO Wishlist(CustomerID, ProductID) VALUES(?,?)");
         $insert_wishlist->execute([$user_id, $pid]);
         $message[] = 'Added to wishlist!';
      }

   }

}

if(isset($_POST['add_to_cart'])){

   if($user_id == ''){
      header('location:user_login.php');
   }else{

      $pid = $_POST['pid'];
      $pid = filter_var($pid, FILTER_SANITIZE_STRING);
      $name = $_POST['name'];
      $name = filter_var($name, FILTER_SANITIZE_STRING);
      $price = $_POST['price'];
      $price = filter_var($price, FILTER_SANITIZE_STRING);
      $image = $_POST['image'];
      $image = filter_var($image, FILTER_SANITIZE_STRING);
      $qty = $_POST['qty'];
      $qty = filter_var($qty, FILTER_SANITIZE_STRING);

      $check_cart_numbers = $conn->prepare("SELECT * FROM Cart WHERE CustomerID = ? AND ProductID = ?");
      $check_cart_numbers->execute([$user_id, $pid]);

      if($check_cart_numbers->rowCount() > 0){
         $message[] = 'Already added to cart!';
      }else{

         $check_wishlist_numbers = $conn->prepare("SELECT * FROM Wishlist WHERE CustomerID = ? AND ProductID = ?");
         $check_wishlist_numbers->execute([$user_id, $pid]);

         if($check_wishlist_numbers->rowCount() > 0){
            $delete_wishlist = $conn->prepare("DELETE FROM Wishlist WHERE CustomerID = ? AND ProductID = ?");
            $delete_wishlist->execute([$user_id, $pid]);
         }

         $insert_cart = $conn->prepare("INSERT INTO Cart(CustomerID, ProductID, Quantity) VALUES(?,?,?)");
         $insert_cart->execute([$user_id, $pid, $qty]);
         $message[] = 'Added to cart!';
         
      }

   }

}

?>