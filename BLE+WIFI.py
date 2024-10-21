import asyncio
import tkinter as tk
from tkinter import messagebox, ttk
from bleak import BleakClient, BleakScanner
import json

# UUIDs do serviço e das características BLE
SERVICE_UUID = "12345678-1234-1234-1234-123456789012"
WIFI_CHAR_UUID = "12345678-1234-1234-1234-123456789013"
SENSOR_CHAR_UUID = "12345678-1234-1234-1234-123456789014"

# Classe da interface gráfica
class WiFiConfigApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Configurar Wi-Fi via BLE")
        self.device = None

        # Campos para SSID e Senha
        self.label_ssid = tk.Label(root, text="SSID:")
        self.label_ssid.grid(row=0, column=0)
        self.entry_ssid = tk.Entry(root)
        self.entry_ssid.grid(row=0, column=1)

        self.label_password = tk.Label(root, text="Senha:")
        self.label_password.grid(row=1, column=0)
        self.entry_password = tk.Entry(root, show="*")
        self.entry_password.grid(row=1, column=1)

        # Campos para Empresa e Placa do caminhão
        self.label_company = tk.Label(root, text="Empresa da Frota:")
        self.label_company.grid(row=2, column=0)
        self.entry_company = tk.Entry(root)
        self.entry_company.grid(row=2, column=1)

        self.label_plate = tk.Label(root, text="Placa do Caminhão:")
        self.label_plate.grid(row=3, column=0)
        self.entry_plate = tk.Entry(root)
        self.entry_plate.grid(row=3, column=1)

        # Combo box para selecionar dispositivos BLE
        self.device_combobox = ttk.Combobox(root)
        self.device_combobox.grid(row=4, column=0, columnspan=2)
        self.device_combobox.set("Selecione um dispositivo BLE")

        # Botões
        self.button_scan = tk.Button(root, text="Procurar Dispositivos", command=self.scan_ble)
        self.button_scan.grid(row=5, column=0, columnspan=2)

        self.button_connect = tk.Button(root, text="Conectar ao BLE", command=self.connect_ble)
        self.button_connect.grid(row=6, column=0, columnspan=2)

        self.button_send = tk.Button(root, text="Enviar Configuração Wi-Fi", command=self.send_credentials)
        self.button_send.grid(row=7, column=0, columnspan=2)

        self.button_receive_data = tk.Button(root, text="Receber Dados do Sensor", command=self.receive_sensor_data)
        self.button_receive_data.grid(row=8, column=0, columnspan=2)

        # Configurar loop para chamada assíncrona do Tkinter
        self.loop = asyncio.get_event_loop()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    async def scan_ble(self):
        devices = await BleakScanner.discover()
        self.device_combobox['values'] = [device.name for device in devices if device.name]
        if not self.device_combobox['values']:
            messagebox.showinfo("Nenhum Dispositivo", "Nenhum dispositivo BLE encontrado.")

    async def connect_ble(self):
        selected_device = self.device_combobox.get()
        if selected_device:
            devices = await BleakScanner.discover()
            for device in devices:
                if device.name == selected_device:
                    self.device = await BleakClient(device).connect()
                    messagebox.showinfo("Sucesso", f"Conectado ao {device.name}")
                    return

            messagebox.showerror("Erro", "Não foi possível conectar ao dispositivo selecionado.")
        else:
            messagebox.showerror("Erro", "Por favor, selecione um dispositivo BLE.")

    async def send_credentials(self):
        ssid = self.entry_ssid.get()
        password = self.entry_password.get()
        company = self.entry_company.get()
        plate = self.entry_plate.get()

        if ssid and password and company and plate and self.device:
            # Criar JSON com SSID, senha, empresa e placa
            credentials = {
                "SSID": ssid,
                "Password": password,
                "Company": company,
                "Plate": plate
            }

            # Enviar as credenciais em formato JSON via BLE
            credentials_json = json.dumps(credentials).encode()
            await self.device.write_gatt_char(WIFI_CHAR_UUID, credentials_json)
            messagebox.showinfo("Sucesso", "Credenciais Wi-Fi e informações da frota enviadas com sucesso!")
        else:
            messagebox.showerror("Erro", "Por favor, preencha todos os campos e conecte ao BLE.")

    async def receive_sensor_data(self):
        if self.device:
            # Ler dados da característica do sensor via BLE
            sensor_data = await self.device.read_gatt_char(SENSOR_CHAR_UUID)

            # Decodificar e exibir os dados recebidos (em formato JSON)
            sensor_data_json = sensor_data.decode()
            sensor_data_dict = json.loads(sensor_data_json)

            temperature = sensor_data_dict.get("Temperature", "N/A")
            humidity = sensor_data_dict.get("Humidity", "N/A")

            messagebox.showinfo("Dados do Sensor", f"Temperatura: {temperature}\nUmidade: {humidity}")
        else:
            messagebox.showerror("Erro", "Por favor, conecte-se ao dispositivo BLE antes de tentar receber os dados.")

    def on_closing(self):
        if self.device:
            asyncio.create_task(self.device.disconnect())
        self.root.destroy()

# Função principal para executar a interface
def main():
    root = tk.Tk()
    app = WiFiConfigApp(root)

    # Adicionar métodos assíncronos como callbacks
    app.button_scan.config(command=lambda: asyncio.create_task(app.scan_ble()))
    app.button_connect.config(command=lambda: asyncio.create_task(app.connect_ble()))
    app.button_send.config(command=lambda: asyncio.create_task(app.send_credentials()))
    app.button_receive_data.config(command=lambda: asyncio.create_task(app.receive_sensor_data()))

    # Iniciar o loop de eventos do Tkinter
    root.mainloop()

if __name__ == "__main__":
    main()
