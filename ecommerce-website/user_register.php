<?php

include 'components/connect.php';

session_start();

if(isset($_SESSION['CustomerID'])){
   $user_id = $_SESSION['CustomerID'];
}else{
   $user_id = '';
};

if(isset($_POST['submit'])){

   $name = $_POST['name'];
   $name = filter_var($name, FILTER_SANITIZE_STRING);
   $email = $_POST['email'];
   $email = filter_var($email, FILTER_SANITIZE_STRING);
   $pass = $_POST['pass'];
   $pass = filter_var($pass, FILTER_SANITIZE_STRING);
   $cpass = $_POST['cpass'];
   $cpass = filter_var($cpass, FILTER_SANITIZE_STRING);
   $gender = $_POST['Gender'];
   $gender = filter_var($gender, FILTER_SANITIZE_STRING);

   $select_user = $conn->prepare("SELECT * FROM Customers WHERE Email = ?");
   $select_user->execute([$email,]);
   $row = $select_user->fetch(PDO::FETCH_ASSOC);

   if($select_user->rowCount() > 0){
      $message[] = 'Email already exists!';
   }else{
      if($pass != $cpass){
         $message[] = 'Confirm password not matched!';
      }else{
         $last_id = $conn->prepare("SELECT CustomerID FROM Customers ORDER BY CustomerID DESC LIMIT 1");
         $last_id->execute();

         if ($last_id->rowCount() > 0) {
            $row = $last_id->fetch(PDO::FETCH_ASSOC);
            $last_id_num = (int)substr(strval($row["customerid"]), 1);
            $next_id_num = $last_id_num + 1;
            $next_id = 'C' . strval($next_id_num);
            }
         else {
            $next_id = "C1";
         }

         $insert_user = $conn->prepare("INSERT INTO Customers(CustomerID, FullName, Email, Password, Gender) VALUES(?,?,?,?,?)");
         $insert_user->execute([$next_id, $name, $email, $cpass,$gender]);
         $message[] = 'Registered successfully. Login now please!';
      }
   }

}

?>

<!DOCTYPE html>
<html lang="en">
<head>
   <meta charset="UTF-8">
   <meta http-equiv="X-UA-Compatible" content="IE=edge">
   <meta name="viewport" content="width=device-width, initial-scale=1.0">
   <title>register</title>
   
   <!-- font awesome cdn link  -->
   <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.1.1/css/all.min.css">

   <!-- custom css file link  -->
   <link rel="stylesheet" href="css/style.css">

</head>
<body>
   
<?php include 'components/user_header.php'; ?>

<section class="form-container">

   <form action="" method="post">
      <h3>register now</h3>
      <input type="text" name="name" required placeholder="enter your username" maxlength="20"  class="box">
      <input type="email" name="email" required placeholder="enter your email" maxlength="50"  class="box" oninput="this.value = this.value.replace(/\s/g, '')">
      <input type="password" name="pass" required placeholder="enter your password" maxlength="20"  class="box" oninput="this.value = this.value.replace(/\s/g, '')">
      <input type="password" name="cpass" required placeholder="confirm your password" maxlength="20"  class="box" oninput="this.value = this.value.replace(/\s/g, '')">
      <p> Gender </p>
      <select name="Gender">
      <option value="Male">Male</option>
      <option value="Female">Female</option>
      </select>
      <input type="submit" value="register now" class="btn" name="submit">
      <p>already have an account?</p>
      <a href="user_login.php" class="option-btn">login now</a>
   </form>

</section>











<?php include 'components/footer.php'; ?>

<script src="js/script.js"></script>

</body>
</html>
