<?php
error_reporting(E_ALL);
ini_set('display_errors', 1);

// MySQL connection info
$host = "localhost";
$user = "your_mqtt_user";
$password = "your_password";
$database = "your_data_base_name";

// Connect to MySQL
$conn = new mysqli($host, $user, $password, $database);

// Check connection
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
}

// Query: latest row for each data_type
$sql = "
    SELECT t1.*
    FROM data_base_table t1
    INNER JOIN (
        SELECT data_type, MAX(fecha) AS latest
        FROM data_base_table
        GROUP BY data_type
    ) t2 ON t1.data_type = t2.data_type AND t1.fecha = t2.latest
    ORDER BY t1.data_type ASC
";

$result = $conn->query($sql);
$values = [];
if ($result && $result->num_rows > 0) {
    while($row = $result->fetch_assoc()) {
        $values[$row["data_type"]] = [
            'value' => floatval($row["value"]),
            'fecha' => $row["fecha"]
        ];
    }
}
$conn->close();

function getCardClass($type, $value) {
    switch($type) {
        case 'temperature':
            if ($value < 0) return 'cold';
            if ($value < 10) return 'chilly';
            if ($value < 20) return 'cool';
            if ($value < 30) return 'warm';
            return 'hot';
        case 'humidity':
            if ($value < 40) return 'dry';
            if ($value <= 60) return 'optimal';
            return 'humid';
        case 'SpO2':
            if ($value >= 90) return 'ok';
            if ($value >= 80) return 'warning';
            return 'danger';
        case 'bpm':
            if ($value < 60) return 'warning';
            if ($value <= 100) return 'ok';
            return 'danger';
        case 'CO concentration':
            if ($value <= 10) return 'ok';
            if ($value <= 30) return 'warning';
            return 'danger';
        default:
            return '';
    }
}
?>
<!DOCTYPE html>
<html>
<head>
    <title>Estación de Salud</title>
    <meta http-equiv="refresh" content="5">
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(to right, #f8fcff, #dbefff);
            padding: 2rem;
            margin: 0;
        }
        h1 {
            text-align: center;
            color: #004d66;
            margin-bottom: 40px;
            font-size: 2.5rem;
        }
        .dashboard {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
            gap: 30px;
            max-width: 1200px;
            margin: auto;
        }
        .card {
            border-radius: 18px;
            padding: 20px;
            height: 180px;
            text-align: center;
            box-shadow: 0 6px 16px rgba(0, 0, 0, 0.1);
            transition: transform 0.3s ease-in-out;
            display: flex;
            flex-direction: column;
            justify-content: center;
            color: white;
        }
        .card:hover {
            transform: scale(1.05);
        }
        .card h2 {
            font-size: 1.3em;
            margin-bottom: 8px;
            color: #fff;
        }
        .card p {
            font-size: 2.2em;
            margin: 0;
        }
        .card small {
            margin-top: 10px;
            font-size: 0.85em;
            color: #eeeeee;
        }
        .cold { background-color: #0277bd; }
        .chilly { background-color: #03a9f4; }
        .cool { background-color: #4fc3f7; }
        .warm { background-color: #ffb300; color: #000; }
        .hot { background-color: #e53935; }
        .dry { background-color: #a1887f; }
        .optimal { background-color: #43a047; }
        .humid { background-color: #26c6da; }
        .ok { background-color: #388e3c; }
        .warning { background-color: #fbc02d; color: #000; }
        .danger {
            background-color: #d32f2f;
            animation: blink 1s infinite;
            position: relative;
        }
        .danger::after {
            content: '⚠ Alerta Crítica';
            position: absolute;
            bottom: 8px;
            left: 0;
            right: 0;
            font-size: 0.9em;
            color: yellow;
        }
        @keyframes blink {
            50% { opacity: 0.5; }
        }
    </style>
</head>
<body>

<h1>Valores Actuales del Paciente</h1>
<div class="dashboard">
    <?php foreach ($values as $type => $data): ?>
        <?php $class = getCardClass($type, $data['value']); ?>
        <div class="card <?= $class ?>">
            <h2><?= htmlspecialchars($type) ?></h2>
            <p><?= htmlspecialchars($data['value']) ?></p>
            <small><?= htmlspecialchars($data['fecha']) ?></small>
        </div>
    <?php endforeach; ?>
</div>

</body>
</html>



