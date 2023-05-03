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
   <title>shop</title>
   
   <!-- font awesome cdn link  -->
   <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.1.1/css/all.min.css">

   <!-- custom css file link  -->
   <link rel="stylesheet" href="css/style.css">

</head>
<body>
   
<?php include 'components/user_header.php'; ?>

<section class="products">

   <h1 class="heading">latest products</h1>

   <div class="box-container">

   <?php
     $select_products = $conn->prepare("SELECT * FROM Products"); 
     $select_products->execute();
     if($select_products->rowCount() > 0){
      while($fetch_product = $select_products->fetch(PDO::FETCH_ASSOC)){
   ?>
      <form action="" method="post" class="box">
         <input type="hidden" name="pid" value="<?= $fetch_product['productid']; ?>">
         <input type="hidden" name="name" value="<?= $fetch_product['productname']; ?>">
         <input type="hidden" name="price" value="<?= $fetch_product['price']; ?>">
         <input type="hidden" name="image" value="<?= $fetch_product['image_1']; ?>">
         <button class="fas fa-heart" type="submit" name="add_to_wishlist"></button>
         <a href="quick_view.php?pid=<?= $fetch_product['productid']; ?>" class="fas fa-eye"></a>
         <img src="uploaded_img/<?= $fetch_product['image_1']; ?>" alt="">
         <div class="name"><?= $fetch_product['productname']; ?></div>
         <div class="flex">
            <div class="price"><span>$</span><?= $fetch_product['price']; ?><span>/-</span></div>
            <input type="number" name="qty" class="qty" min="1" max="<?php echo $fetch_product['itemsinstock']; ?>" onkeypress="if(this.value.length == 2) return false;" value="1">
         </div>
         <?php
            if($fetch_product['itemsinstock']>0)
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
      </form>
   <?php
      }
   }else{
      echo '<p class="empty">No products found!</p>';
   }
   ?>

   </div>

</section>


<?php include 'components/footer.php'; ?>

<script src="js/script.js"></script>

</body>
</html>