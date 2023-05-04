<?php

require_once __DIR__ . '/../vendor/autoload.php';

$dotenv = Dotenv\Dotenv::createImmutable(__DIR__);
$dotenv->load();

$database_host = $_ENV['DATABASE_HOST'];
$database_port = $_ENV['DATABASE_PORT'];
$database_user = $_ENV['DATABASE_USER'];
$database_password = $_ENV['DATABASE_PASSWORD'];
$database_name = $_ENV['DATABASE_NAME'];

$dsn = "pgsql:host=$database_host;port=$database_port;dbname=$database_name";

$conn = new PDO($dsn, $database_user, $database_password);

?>