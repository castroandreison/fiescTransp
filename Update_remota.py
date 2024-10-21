import random
import time
from paho.mqtt import client as mqtt_client

broker = 'broker.emqx.io'  # Endereço do broker MQTT
port = 1883                  # Porta do broker MQTT
topic = "redefrigo/firmware"  # Tópico para enviar a URL do firmware
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'emqx'           # Usuário do MQTT
password = 'public'         # Senha do MQTT

def connect_mqtt() -> mqtt_client:
    """Conecta ao broker MQTT e retorna o cliente."""
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Conectado ao broker MQTT!")
        else:
            print(f"Falha na conexão, código de retorno: {rc}")

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def publish_firmware_url(client: mqtt_client, firmware_url: str):
    """Publica a URL do firmware no tópico definido."""
    result = client.publish(topic, firmware_url)
    status = result[0]
    if status == 0:
        print(f"Mensagem enviada para o tópico '{topic}': {firmware_url}")
    else:
        print(f"Falha ao enviar mensagem para o tópico '{topic}'")

def run():
    client = connect_mqtt()
    client.loop_start()  # Inicia o loop para manter a conexão

    # Substitua pela URL do firmware que deseja enviar
    firmware_url = "https://drive.google.com/file/d/13qjx3YfCWQ3lZBaznzplG5AMXG2qZLCM/view?usp=drive_link/CodigoFiesc.ino.bin"  # Altere para uma URL acessível
    # Exemplo de URL local (apenas para testes): firmware_url = "http://localhost:8000/firmware.bin"

    try:
        while True:
            publish_firmware_url(client, firmware_url)  # Publica a URL do firmware
            time.sleep(10)  # Espera 10 segundos antes de enviar novamente
    except KeyboardInterrupt:
        print("Encerrando o cliente MQTT...")
    finally:
        client.loop_stop()  # Para o loop
        client.disconnect()  # Desconecta do broker

if __name__ == '__main__':
    run()
