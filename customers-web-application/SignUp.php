<?php
    //connect to db
    $conn=mysqli_connect('localhost','root','','whr');
    $error_field=array();
    if(!(isset($_POST['Name'])&& !empty($_POST['Name'])))
        $error_field[]='Name';
    if(!(isset($_POST['Email'])&& filter_input(INPUT_POST,'Email',FILTER_VALIDATE_EMAIL)))
       $error_field[]='Email';
    if(!(isset($_POST['Password'])&& strlen($_POST['Password'])>5))
       $error_field[]='Password';
    
    if($error_field) {
        header("Location:FirstPage.html?error_fields=".implode(",",$error_field));
           exit(); 
    }
    if(!$conn)
    { echo mysqli_connect_error();
      exit;}
    //Escape any special characters to avoid SQL injection
    $name=mysqli_escape_string($conn,$_POST['Name']);
    $email=mysqli_escape_string($conn,$_POST['Email']);
    $password=mysqli_escape_string($conn,$_POST['Password']);
    //insert data
$query = "insert into `users` (`Name`,`Email`,`Password`) values ('".$name."','".$email."','".$password."')";
    if(mysqli_query($conn,$query))
    {
        echo "Thank You For Signing Up";
    }
    else
    {
        echo $query;
        echo mysqli_error($conn);
    }
?>