<?php

include 'components/connect.php';

session_start();

if(isset($_SESSION['CustomerID'])){
   $user_id = $_SESSION['CustomerID'];
}else{
   $user_id = '';
};

if(isset($_POST['send'])){

   $email = $_POST['email'];
   $email = filter_var($email, FILTER_SANITIZE_STRING);
   $number = $_POST['number'];
   $number = filter_var($number, FILTER_SANITIZE_STRING);
   $msg = $_POST['msg'];
   $msg = filter_var($msg, FILTER_SANITIZE_STRING);

   $select_message = $conn->prepare("SELECT * FROM Customer_Services AS CS
                                       INNER JOIN Customers AS C
                                          ON C.CustomerID = CS.CustomerID
                                       WHERE C.Email = ? AND CS.Message = ?");
   $select_message->execute([$email, $msg]);

   if($select_message->rowCount() > 0){
      $message[] = 'already sent message!';
   }else{

      $num_of_msgs = $conn->prepare("SELECT COUNT(MessageID) AS num_msgs FROM Customer_Services");
      $num_of_msgs->execute();
      $result = $num_of_msgs->fetch(PDO::FETCH_ASSOC);
      $num_of_msgs = $result["num_msgs"];
      $msg_id = $num_of_msgs + 1;
      $msg_id = 'M' . strval($msg_id);

      $insert_message = $conn->prepare("INSERT INTO Customer_Services(MessageID, CustomerID, PhoneNumber, Message) VALUES(?,?,?,?)");
      $insert_message->execute([$msg_id, $user_id, $number, $msg]);

      $message[] = 'sent message successfully!';

   }

}


?>

<!DOCTYPE html>
<html lang="en">
<head>
   <meta charset="UTF-8">
   <meta http-equiv="X-UA-Compatible" content="IE=edge">
   <meta name="viewport" content="width=device-width, initial-scale=1.0">
   <title>contact</title>
   
   <!-- font awesome cdn link  -->
   <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.1.1/css/all.min.css">

   <!-- custom css file link  -->
   <link rel="stylesheet" href="css/style.css">

</head>
<body>
   
<?php include 'components/user_header.php'; ?>

<section class="contact">

   <form action="" method="post">
      <h3>get in touch</h3>
      <input type="text" name="name" placeholder="enter your name" required maxlength="20" class="box">
      <input type="email" name="email" placeholder="enter your email" required maxlength="50" class="box">
      <input type="number" name="number" min="0" max="9999999999" placeholder="enter your number" required onkeypress="if(this.value.length == 10) return false;" class="box">
      <textarea name="msg" class="box" placeholder="enter your message" cols="30" rows="10"></textarea>
      <input type="submit" value="send message" name="send" class="btn">
   </form>

</section>













<?php include 'components/footer.php'; ?>

<script src="js/script.js"></script>

</body>
</html>