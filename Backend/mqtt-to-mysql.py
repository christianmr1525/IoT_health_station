#!/usr/bin/env python3

import mysql.connector
import paho.mqtt.client as mqtt

# MySQL connection config
mysql_host = "localhost"
mysql_user = "your_mqtt_user"
mysql_password = "your_password"
mysql_database = "your_data_base_name"

# MQTT config
mqtt_broker = "localhost"
mqtt_topic_subscribe = "/Clase"
mqtt_topic_publish = "DB_last_values"

# MySQL connection config as a dictionary
db_config = {
    'host': mysql_host,
    'user': mysql_user,
    'password': mysql_password,
    'database': mysql_database
}

def publish_latest_values():
    """Query and publish the latest value of each data_type to DB_last_values."""
    try:
        conn = mysql.connector.connect(**db_config)
        cursor = conn.cursor()

        # Correct SQL query (avoids ONLY_FULL_GROUP_BY error)
        sql = """
        SELECT t1.data_type, t1.value
        FROM data_base_table t1
        JOIN (
            SELECT data_type, MAX(fecha) AS max_fecha
            FROM data_base_table
            GROUP BY data_type
        ) t2
        ON t1.data_type = t2.data_type AND t1.fecha = t2.max_fecha;
        """
        cursor.execute(sql)
        rows = cursor.fetchall()

        # Format message
        message_lines = []
        for data_type, value in rows:
            message_lines.append(f"{data_type},{value}")
        message = "\n".join(message_lines)

        # Publish to MQTT
        client.publish(mqtt_topic_publish, message)
        print(f"Published latest values to '{mqtt_topic_publish}':\n{message}")

    except mysql.connector.Error as err:
        print(f"Failed to fetch or publish latest values: {err}")
    finally:
        if conn:
            conn.close()

def on_message(client, userdata, msg):
    message = msg.payload.decode()
    print(f"Received message: {message}")

    try:
        data_type, value = message.split(",")
        value = float(value)
    except ValueError:
        print(f"Invalid message format: {message}")
        return

    try:
        conn = mysql.connector.connect(**db_config)
        cursor = conn.cursor()

        # Insert into database
        sql = """
        INSERT INTO data_base_table (data_type, value, fecha)
        VALUES (%s, %s, NOW())
        """
        cursor.execute(sql, (data_type, value))
        conn.commit()
        print("Message inserted into database.")

        # After inserting, publish the latest values to ESP32
        publish_latest_values()

    except mysql.connector.Error as err:
        print(f"Failed to insert message: {err}")
    finally:
        if conn:
            conn.close()

# Setup MQTT client
client = mqtt.Client()
client.on_message = on_message

client.connect(mqtt_broker, 1883, 60)
client.subscribe(mqtt_topic_subscribe)

print(f"Subscribed to MQTT topic '{mqtt_topic_subscribe}' and listening...")

# Start MQTT loop
client.loop_forever()


