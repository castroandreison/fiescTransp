import random
import time
import json
from paho.mqtt import client as mqtt_client

# Configurações do broker MQTT
broker = 'broker.emqx.io'
port = 1883
topic = "redefrigo/sensor_data"  # Tópico para ler os dados do sensor
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'emqx'
password = 'public'

def connect_mqtt() -> mqtt_client:
    """Conecta ao broker MQTT e retorna o cliente."""
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Conectado ao broker MQTT!")
            client.subscribe(topic)  # Inscreve no tópico
        else:
            print(f"Falha na conexão, código de retorno: {rc}")

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.on_message = on_message  # Define a função de callback para mensagens recebidas
    client.connect(broker, port)
    return client

def on_message(client, userdata, msg):
    """Callback para mensagens recebidas."""
    try:
        # Decodifica a mensagem recebida
        message = msg.payload.decode()
        # Carrega a mensagem JSON
        data = json.loads(message)
        # Imprime os dados
        print(f"Mensagem recebida no tópico '{msg.topic}': {data}")
    except json.JSONDecodeError as e:
        print(f"Erro ao decodificar a mensagem JSON: {e}")

def run():
    client = connect_mqtt()
    client.loop_start()  # Inicia o loop para manter a conexão

    try:
        while True:
            time.sleep(1)  # Aguarda novas mensagens
    except KeyboardInterrupt:
        print("Encerrando o cliente MQTT...")
    finally:
        client.loop_stop()  # Para o loop
        client.disconnect()  # Desconecta do broker

if __name__ == '__main__':
    run()
