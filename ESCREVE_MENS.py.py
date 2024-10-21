import tkinter as tk
from tkinter import messagebox
import paho.mqtt.client as mqtt
import random

# Configurações do broker MQTT
broker = 'broker.emqx.io'
port = 1883
topic = "redefrigo/Resp"
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'emqx'
password = 'public'

# Função para enviar a mensagem selecionada
def send_message():
    selected_message = message_var.get()
    if selected_message:
        result = client.publish(topic, selected_message)
        # Verifica se a mensagem foi publicada com sucesso
        if result.rc == 0:
            messagebox.showinfo("Mensagem Enviada", f"Mensagem enviada: {selected_message}")
        else:
            messagebox.showwarning("Erro", "Falha ao enviar a mensagem.")
    else:
        messagebox.showwarning("Selecione uma Mensagem", "Por favor, selecione uma mensagem para enviar.")

# Callback para conexão MQTT
def on_connect(client, userdata, flags, rc):
    print(f"Conectado com o resultado: {rc}")

# Configurações do cliente MQTT
client = mqtt.Client(client_id)
client.username_pw_set(username, password)
client.on_connect = on_connect

# Conectar ao broker MQTT
try:
    client.connect(broker, port, 60)
except Exception as e:
    print(f"Erro ao conectar ao broker: {e}")

# Loop para manter a conexão com o broker
def mqtt_loop():
    client.loop_start()  # Inicia o loop de processamento do cliente MQTT

# Criação da interface gráfica
root = tk.Tk()
root.title("Sistema de apoio ao motorista")

# Variável para armazenar a mensagem selecionada
message_var = tk.StringVar()

# Lista de mensagens
messages = [
    "Calibrar pneus",
    "Deve-se aguardar o tempo limite para retorno a direção",
    "Caminhão desligado devido a ingerir álcool",
    "Limite de horas dirigindo excedido"
    "Manobras perigosas, curvas que comprometem a seguraça"
]

# Criação do menu de seleção
message_label = tk.Label(root, text="Selecione uma Mensagem:")
message_label.pack(pady=10)

for msg in messages:
    rb = tk.Radiobutton(root, text=msg, variable=message_var, value=msg)
    rb.pack(anchor=tk.W)

# Botão para enviar a mensagem
send_button = tk.Button(root, text="Enviar", command=send_message)
send_button.pack(pady=20)

mqtt_loop()  # Inicia o loop MQTT

# Loop principal da interface
root.mainloop()

# Desconectar ao sair
client.loop_stop()  # Para o loop de processamento do cliente MQTT
client.disconnect()  # Desconecta do broker
