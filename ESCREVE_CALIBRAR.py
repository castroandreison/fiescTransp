import tkinter as tk
from tkinter import messagebox
import paho.mqtt.client as mqtt
import random

# Configurações do broker MQTT
broker = 'broker.emqx.io'
port = 1883
topic = "redefrigo/tpms_control"
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'emqx'
password = 'public'

# Função para enviar a mensagem selecionada
def send_message():
    selected_message = message_var.get()
    if selected_message:
        client.publish(topic, selected_message)
        messagebox.showinfo("Mensagem Enviada", f"Mensagem enviada: {selected_message}")
    else:
        messagebox.showwarning("Selecione uma Mensagem", "Por favor, selecione uma mensagem para enviar.")

# Configurações do cliente MQTT
client = mqtt.Client(client_id)
client.username_pw_set(username, password)

# Conectar ao broker MQTT
try:
    client.connect(broker, port, 60)
except Exception as e:
    print(f"Erro ao conectar ao broker: {e}")

# Criação da interface gráfica
root = tk.Tk()
root.title("Envio de Mensagens MQTT")

# Variável para armazenar a mensagem selecionada
message_var = tk.StringVar()

# Lista de mensagens
messages = [
    "Calibrar pneus",
    "Parar de roldar",
    "Caminhão desligado devido a ingerir álcool",
    "Limite de horas dirigindo excedido"
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

# Loop principal da interface
root.mainloop()

# Desconectar ao sair
client.disconnect()
