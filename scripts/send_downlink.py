import paho.mqtt.client as mqtt
import base64
import json
import time
import sys

# Configuración TTN
APP_ID = "tu_app_id"
DEVICE_ID = "tu_device_id"
API_KEY = "tu_api_key"  # API key con permisos de downlink

# Servidor MQTT
REGION = "eu1"  # Cambia según tu región: eu1, nam1, au1, etc.
MQTT_SERVER = f"{REGION}.cloud.thethings.network"
MQTT_PORT = 1883

# Callback de conexión
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Conectado a TTN correctamente")
    else:
        print(f"Error de conexión: {rc}")

# Callback para mensajes
def on_message(client, userdata, msg):
    print(f"Mensaje recibido en {msg.topic}: {msg.payload}")

# Crear cliente MQTT
client = mqtt.Client()
client.username_pw_set(APP_ID, API_KEY)
client.on_connect = on_connect
client.on_message = on_message

# Conectar
client.connect(MQTT_SERVER, MQTT_PORT, 60)
client.loop_start()

# Enviar downlink
def send_downlink(payload_hex, port=1, confirmed=True):
    # Convertir hex a bytes y luego a base64
    payload_bytes = bytes.fromhex(payload_hex.replace(" ", ""))
    payload_b64 = base64.b64encode(payload_bytes).decode()
    
    # Construir mensaje
    downlink = {
        "downlinks": [{
            "frm_payload": payload_b64,
            "f_port": port,
            "priority": "NORMAL",
            "confirmed": confirmed
        }]
    }
    
    # Publicar mensaje
    topic = f"v3/{APP_ID}/devices/{DEVICE_ID}/down/push"
    client.publish(topic, json.dumps(downlink))
    
    print(f"Enviando downlink: {payload_hex}")
    print(f"Topic: {topic}")
    print(f"Mensaje: {json.dumps(downlink)}")

# Enviar payload desde argumentos de línea de comandos
if len(sys.argv) > 1:
    payload = sys.argv[1]
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    send_downlink(payload, port)
    print(f"Enviando: {payload} al puerto {port}")
else:
    # O usar un payload predeterminado
    send_downlink("00 00 00 00", 1)

# Mantener activo para recibir respuestas
time.sleep(5)
client.loop_stop()
client.disconnect()
