<?php
$host = "0.0.0.0"      // your Azure Server IP
$user = "your_mqtt_user";
$pass = "your_password";
$dbname = "your_data_base_name";

// Create connection
$conn = new mysqli($host, $user, $pass, $dbname);

// Check connection
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
}
?>
