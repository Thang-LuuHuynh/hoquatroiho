# cloud_sub.py
import ssl
import paho.mqtt.client as mqtt

# ==== THÔNG SỐ EMQX CLOUD CỦA BẠN ====
BROKER   = "o7710815.ala.asia-southeast1.emqxsl.com"  # nhớ là chữ 'o', không phải số 0
PORT     = 8883                                      # MQTT over TLS/SSL port
USERNAME = "admin"                                   # user bạn tạo trong EMQX Cloud
PASSWORD = "Blank14!"                                  # mật khẩu tương ứng
TOPIC    = "KEPServerEX-iotgateway/#"                # sửa theo topic KEPServer gửi lên

# ==== CALLBACKS ====
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("✅ Đã kết nối EMQX Cloud")
        client.subscribe(TOPIC, qos=1)
        print(f"📡 Đang subscribe topic: {TOPIC}")
    else:
        print("❌ Kết nối lỗi, rc =", rc)

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
    except UnicodeDecodeError:
        payload = msg.payload
    print(f"[MSG] {msg.topic} -> {payload}")

# ==== TẠO CLIENT & CẤU HÌNH TLS ====
client = mqtt.Client(
    client_id="Siu",
    protocol=mqtt.MQTTv5  # hoặc MQTTv311 đều được
)

client.username_pw_set(USERNAME, PASSWORD)

# Nếu bạn CHƯA dùng file CA thì tạm thời bỏ verify (test cho nhanh)
client.tls_set(cert_reqs=ssl.CERT_NONE)
client.tls_insecure_set(True)

# Nếu bạn đã tải file CA về (vd: emqxsl-ca.crt) thì nên dùng:
# client.tls_set(ca_certs="emqxsl-ca.crt", cert_reqs=ssl.CERT_REQUIRED)
# client.tls_insecure_set(False)

client.on_connect = on_connect
client.on_message = on_message

print("🔌 Đang kết nối tới EMQX Cloud...")
client.connect(BROKER, PORT, keepalive=60)

# Vòng lặp nhận dữ liệu
client.loop_forever()
