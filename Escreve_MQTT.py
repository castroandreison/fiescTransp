import random
import time
from threading import Timer

from paho.mqtt import client as mqtt_client


broker = 'broker.emqx.io'
port = 1883
topic = "redefrigo/firmware"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'emqx'
password = 'public'


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print(f"Failed to connect, return code {rc}")

    # Criar cliente MQTT com a versão adequada do protocolo, sem 'callback_api_version'
    client = mqtt_client.Client(client_id, protocol=mqtt_client.MQTTv311)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def publish(client):
    msg_count = "01" + "03" + "02" + "00" + "01" + "2A" + "CA"
    while True:
        time.sleep(3)
        msg = f"{msg_count}"
        result = client.publish(topic, msg)
        status = result.rc  # Obter o código de retorno
        if status == 0:
            print(f"Escrevendo - Dado `{msg}` Tópico: `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")


def run():
    client = connect_mqtt()
    client.loop_start()
    publish(client)


if __name__ == '__main__':
    run()
