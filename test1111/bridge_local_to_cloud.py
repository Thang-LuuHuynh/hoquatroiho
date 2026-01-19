import ssl
import time
import paho.mqtt.client as mqtt

# ====== THÔNG TIN BROKER LOCAL (Mosquitto) ======
LOCAL_HOST = "127.0.0.1"
LOCAL_PORT = 1883
LOCAL_TOPIC = "KEPServerEX-iotgateway/#"

# ====== THÔNG TIN EMQX CLOUD ======
CLOUD_HOST = "o7710815.ala.asia-southeast1.emqxsl.com"   # địa chỉ của bạn
CLOUD_PORT = 8883
CLOUD_USERNAME = "admin"        # user EMQX Cloud
CLOUD_PASSWORD = "Blank14!"  # password EMQX Cloud
CA_FILE = r"D:\duong_dan\emqxsl-ca.crt"   # file CA bạn đã tải trong Connection Guide

# --- client kết nối EMQX Cloud ---
cloud_client = mqtt.Client(client_id="bridge_to_emqx")
cloud_client.username_pw_set(CLOUD_USERNAME, CLOUD_PASSWORD)
cloud_client.tls_set(
    ca_certs=CA_FILE,
    certfile=None,
    keyfile=None,
    cert_reqs=ssl.CERT_REQUIRED,
    tls_version=ssl.PROTOCOL_TLS_CLIENT,
    ciphers=None
)
cloud_client.tls_insecure_set(False)

def connect_cloud():
    print("👉 Đang kết nối tới EMQX Cloud...")
    cloud_client.connect(CLOUD_HOST, CLOUD_PORT, keepalive=60)
    cloud_client.loop_start()
    print("✅ Đã kết nối EMQX Cloud")

connect_cloud()

# --- client kết nối Mosquitto local ---
local_client = mqtt.Client(client_id="bridge_from_local")

def on_local_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ Đã kết nối Mosquitto local")
        print(f"👉 Subscribe topic: {LOCAL_TOPIC}")
        client.subscribe(LOCAL_TOPIC, qos=1)
    else:
        print("❌ Kết nối Mosquitto local lỗi, rc =", rc)

def on_local_message(client, userdata, msg):
    try:
        topic = msg.topic
        payload = msg.payload
        # In ra xem
        print(f"[LOCAL] {topic} -> {payload}")

        # publish lên EMQX Cloud với cùng topic
        cloud_client.publish(topic, payload, qos=1)
        # print(f"   ↳ [CLOUD] forward {topic}")
    except Exception as e:
        print("Lỗi khi forward:", e)

local_client.on_connect = on_local_connect
local_client.on_message = on_local_message

print("👉 Đang kết nối Mosquitto local...")
local_client.connect(LOCAL_HOST, LOCAL_PORT, keepalive=60)

# vòng lặp cho client local
local_client.loop_forever()
