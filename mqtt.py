import paho.mqtt.client as mqtt
import time
import random

# MQTT 伺服器設定
broker = "broker.hivemq.com"
port = 1883
topic = "ccvs/random"

# 建立 MQTT 客戶端
client = mqtt.Client()

# 連接到 MQTT Broker
def connect_broker():
    try:
        client.connect(broker, port, 60)
        print(f"已連接到 MQTT Broker: {broker}")
    except Exception as e:
        print(f"無法連接到 MQTT Broker: {e}")

# 定義上傳數值的函數
def publish_value():
    while True:
        value = random.randint(0, 100)  # 產生隨機數值
        payload = f"valie: {value}"
        result = client.publish(topic, payload)
        status = result.rc

        if status == 0:
            print(f"已成功發佈: {payload}")
        else:
            print(f"發佈失敗，狀態碼: {status}")
        
        time.sleep(5)  # 每五秒發佈一次

if __name__ == "__main__":
    connect_broker()
    publish_value()

