<?php

include 'components/connect.php';

session_start();

if(isset($_SESSION['user_id'])){
   $user_id = $_SESSION['user_id'];
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
   <title>About</title>

   <link rel="stylesheet" href="https://unpkg.com/swiper@8/swiper-bundle.min.css" />
   
   <!-- font awesome cdn link  -->
   <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.1.1/css/all.min.css">

   <!-- custom css file link  -->
   <link rel="stylesheet" href="css/style.css">

</head>
<body>
   
<?php include 'components/user_header.php'; ?>

<section class="about">

   <div class="row">

      <div class="image">
         <img src="images/about-img.svg" alt="">
      </div>

      <div class="content">
         <h3>The Warehouse Robot's Mini Shop</h3>
         <p>Our Website Is For Our Graduation Project For 2023.</p>
         <a href="contact.php" class="btn">contact us</a>
      </div>

   </div>

</section>

<section class="reviews">
   
   <h1 class="heading">Graduation Project Team</h1>

   <div class="swiper reviews-slider">

   <div class="swiper-wrapper">

      <div class="swiper-slide slide">
         <img src="images/af.jpg" alt="">
         
         <h3>Ahmed Fakhr</h3>
         <p>Embedded Engineer</p>
      </div>
      <div class="swiper-slide slide">
         <img src="images/am.jpg" alt="">
   
         <h3>Aya Hossam</h3>
         <p>Software Engineer</p>
      </div>

      <div class="swiper-slide slide">
         <img src="images/ew.jpg" alt="">
         
         <h3>Emad Wagdi</h3>
         <p>Cloud Engineer</p>
      </div>

      <div class="swiper-slide slide">
         <img src="images/ea.jpg" alt="">
        
         <h3>Eslam Ashraf</h3>
         <p>Embedded Engineer</p>
      </div>

      <div class="swiper-slide slide">
         <img src="images/ed.jpg" alt="">

         <h3>Eslam Dyab</h3>
         <p>Software Engineer</p>
      </div>
      


      <div class="swiper-slide slide">
         <img src="images/g.jpg" alt="">

         <h3>Gehad ElKoumy</h3>
         <p>Embedded Engineer</p>
      </div>
      <div class="swiper-slide slide">
         <img src="images/j.jpg" alt="">

         <h3>Jessica Emad </h3>
         <p>Cloud Engineer</p>
      </div>

      <div class="swiper-slide slide">
         <img src="images/mm.jpg" alt="">
      
         <h3>Menna Mamdouh</h3>
         <p>Data Engineer</p>
      </div>

      

   </div>
   
   <div class="swiper-pagination"></div>

   </div>

</section>









<?php include 'components/footer.php'; ?>

<script src="https://unpkg.com/swiper@8/swiper-bundle.min.js"></script>

<script src="js/script.js"></script>

<script>

var swiper = new Swiper(".reviews-slider", {
   loop:true,
   spaceBetween: 20,
   pagination: {
      el: ".swiper-pagination",
      clickable:true,
   },
   breakpoints: {
      0: {
        slidesPerView:1,
      },
      768: {
        slidesPerView: 2,
      },
      991: {
        slidesPerView: 3,
      },
   },
});

</script>

</body>
</html>
