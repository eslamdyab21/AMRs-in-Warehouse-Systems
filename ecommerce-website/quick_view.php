<?php

include 'components/connect.php';

session_start();

if(isset($_SESSION['CustomerID'])){
   $user_id = $_SESSION['CustomerID'];
}else{
   $user_id = '';
};

include 'components/wishlist_cart.php';

?>

<!DOCTYPE html>
<html lang="en">
<head>
   <meta charset="UTF-8">
   <meta http-equiv="X-UA-Compatible" content="IE=edge">
   <meta name="viewport" content="width=device-width, initial-scale=1.0">
   <title>quick view</title>
   
   <!-- font awesome cdn link  -->
   <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.1.1/css/all.min.css">

   <!-- custom css file link  -->
   <link rel="stylesheet" href="css/style.css">

</head>
<body>
   
<?php include 'components/user_header.php'; ?>

<section class="quick-view">

   <h1 class="heading">quick view</h1>

   <?php
     $pid = $_GET['pid'];
     $select_products = $conn->prepare("SELECT * FROM Products WHERE ProductID = ?"); 
     $select_products->execute([$pid]);
     if($select_products->rowCount() > 0){
     $fetch_product = $select_products->fetch(PDO::FETCH_ASSOC)
   ?>
   <form action="" method="post" class="box">
      <input type="hidden" name="pid" value="<?= $fetch_product['productid']; ?>">
      <input type="hidden" name="name" value="<?= $fetch_product['productname']; ?>">
      <input type="hidden" name="price" value="<?= $fetch_product['price']; ?>">
      <input type="hidden" name="image" value="<?= $fetch_product['image_1']; ?>">
      <div class="row">
         <div class="image-container">
            <div class="main-image">
               <img src="uploaded_img/<?= $fetch_product['image_1']; ?>" alt="">
            </div>
            <div class="sub-image">
               <img src="uploaded_img/<?= $fetch_product['image_1']; ?>" alt="">
               <img src="uploaded_img/<?= $fetch_product['image_2']; ?>" alt="">
               <img src="uploaded_img/<?= $fetch_product['image_3']; ?>" alt="">
            </div>
         </div>
         <div class="content">
            <div class="name"><?= $fetch_product['productname']; ?></div>
            <div class="flex">
               <div class="price"><span>$</span><?= $fetch_product['price']; ?><span>/-</span></div>
               <input type="number" name="qty" class="qty" min="1" max="<?php echo $fetch_product['itemsinstock']; ?>" onkeypress="if(this.value.length == 2) return false;" value="1">
            </div>
            <div class="details"><?= $fetch_product['description']; ?></div>
            <div class="flex-btn">

            <?php
               if($fetch_product['itemsinstock'] > 0)
               { ?>
                  <input type="submit" value="add to cart" class="btn" name="add_to_cart">
                  <?php
               }
               else  {
                  ?>
                  <h1 class="btn"> out of stock </h1>
                  <?php
               }
            ?>
               <input class="option-btn" type="submit" name="add_to_wishlist" value="add to wishlist">
            </div>
         </div>
      </div>
   </form>
   <?php
   }else{
      echo '<p class="empty">No products added yet!</p>';
   }
   ?>

</section>













<?php include 'components/footer.php'; ?>

<script src="js/script.js"></script>

</body>
</html>
