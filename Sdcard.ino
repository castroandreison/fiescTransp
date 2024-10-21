#include <WiFi.h>              // Biblioteca para conexão Wi-Fi
#include <PubSubClient.h>      // Biblioteca para MQTT
#include <DHT.h>               // Biblioteca para sensores DHT
#include <Update.h>            // Biblioteca para atualizações OTA
#include <ArduinoJson.h>       // Biblioteca para manipulação de JSON
#include <ModbusMaster.h>      // Biblioteca para Modbus
#include <SD.h>                // Biblioteca para cartão SD

// Configurações da rede Wi-Fi
const char* ssid = "CastroNet";                   // SSID da rede Wi-Fi
const char* password = "castroeribas";            // Senha da rede Wi-Fi

// Configurações do MQTT
const char* mqttServer = "broker.emqx.io";        // Endereço do broker MQTT
const int mqttPort = 1883;                         // Porta do broker MQTT
const char* mqttUser = "emqx";                     // Usuário do MQTT
const char* mqttPassword = "public";                // Senha do MQTT

// Definições dos pinos dos sensores
#define DHTPIN 23                // Pino do sensor DHT
#define DHTTYPE DHT22            // Tipo do sensor DHT (DHT22 para AM2301)
#define TRUCK_ID "ABC123"        // ID da placa do caminhão
#define FROTA_ID "TransFiesc"    // ID da frota
#define KWH_COST 0.94            // Custo do kWh em centavos

// Definições do Modbus
#define MODBUS_SERIAL_RX 16      // Pino RX do Modbus
#define MODBUS_SERIAL_TX 17      // Pino TX do Modbus

#define SD_CS 5                  // Pino de seleção do cartão SD (ajuste conforme necessário)

DHT dht(DHTPIN, DHTTYPE);         // Inicializa o sensor DHT

WiFiClient espClient;             // Cliente Wi-Fi
PubSubClient client(espClient);   // Cliente MQTT
ModbusMaster node;                // Instância do Modbus

void setup() {
    Serial.begin(115200);  // Inicializa a comunicação serial
    dht.begin();           // Inicializa o sensor DHT
    Serial1.begin(9600, SERIAL_8N1, MODBUS_SERIAL_RX, MODBUS_SERIAL_TX); // Configuração da Serial para Modbus
    node.begin(1, Serial1); // ID do dispositivo Modbus (substitua pelo ID correto)

    // Inicializa o cartão SD
    if (!SD.begin(SD_CS)) {
        Serial.println("Falha ao inicializar o cartão SD!");
        return;
    }
    Serial.println("Cartão SD inicializado.");

    setupWiFi();           // Conecta ao Wi-Fi
    client.setServer(mqttServer, mqttPort);  // Configura o servidor MQTT
    client.setCallback(callback);             // Define o callback para mensagens MQTT
}

// Função para conectar ao Wi-Fi
void setupWiFi() {
    Serial.print("Conectando ao WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print("."); // Indica que está tentando conectar
    }
    Serial.println("Conectado ao WiFi!");
}

// Função para conectar ao MQTT
void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Conectando ao MQTT...");
        if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
            Serial.println("Conectado ao MQTT!");
            client.subscribe("redefrigo/firmware"); // Inscreve no tópico de firmware
        } else {
            Serial.print("Falha na conexão ao MQTT, estado: ");
            Serial.print(client.state());
            delay(2000); // Espera antes de tentar reconectar
        }
    }
}

// Função para fazer a atualização do firmware
void updateFirmware(const char* firmwareUrl) {
    // (Código existente permanece aqui)
}

// Função para ler dados do sensor DHT e publicar no MQTT
void readDHTSensor() {
    float humidity = dht.readHumidity();    // Lê a umidade
    float temperature = dht.readTemperature(); // Lê a temperatura em Celsius

    // Verifica se a leitura falhou
    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Falha ao ler o sensor DHT!");
    } else {
        // Cria um objeto JSON
        DynamicJsonDocument doc(200);
        doc["frota_ID"] = FROTA_ID;  // Adiciona o ID da frota
        doc["truck_id"] = TRUCK_ID;  // Adiciona o ID do caminhão
        doc["humidity"] = humidity;
        doc["temperature"] = temperature;

        // Serializa o objeto JSON para uma string
        String jsonString;
        serializeJson(doc, jsonString);

        // Publica os dados no tópico MQTT
        client.publish("redefrigo/sensor_data", jsonString.c_str());
        
        // Imprime os dados no monitor serial
        Serial.print("Publicando no MQTT: ");
        Serial.println(jsonString);

        // Armazena os dados no cartão SD
        storeDataOnSD("DHTData.txt", jsonString);
    }
}

// Função para armazenar dados no cartão SD
void storeDataOnSD(const char* filename, const String& data) {
    File file = SD.open(filename, FILE_APPEND); // Abre o arquivo para escrita
    if (file) {
        file.println(data); // Adiciona os dados ao arquivo
        file.close(); // Fecha o arquivo
        Serial.println("Dados armazenados no cartão SD.");
    } else {
        Serial.println("Erro ao abrir o arquivo para armazenar dados.");
    }
}

// Função para ler dados simulados do GY-87
void readGY87Sensors() {
    // (Código existente permanece aqui)
    // Adicione a chamada para armazenar dados no cartão SD, se necessário
}

// Função para ler dados simulados do sensor MQ-3
void readMQ3Sensor() {
    // (Código existente permanece aqui)
}

// Função para ler dados simulados do sensor SCT013
void readSCT013Sensor() {
    // (Código existente permanece aqui)
}

// Função para ler dados do Modbus
void readModbusData() {
    // (Código existente permanece aqui)
}

// Função para ler dados simulados do TPMS
void readTPMSSensor() {
    // (Código existente permanece aqui)
}

void callback(char* topic, byte* payload, unsigned int length) {
    // (Código existente permanece aqui)
}

void loop() {
    if (!client.connected()) {
        reconnectMQTT();  // Reconecta se não estiver conectado
    }
    
    client.loop();      // Mantém a conexão MQTT viva
    
    // Lê e publica os dados do sensor DHT
    readDHTSensor();
    
    // Lê e publica os dados do GY-87
    readGY87Sensors();

    // Lê e publica os dados do sensor MQ-3
    readMQ3Sensor();

    // Lê e publica os dados do sensor SCT013
    readSCT013Sensor();

    // Lê os dados do Modbus
    readModbusData();

    // Lê e publica os dados do TPMS a cada 10 segundos
    readTPMSSensor();

    delay(10000); // Espera 10 segundos antes da próxima leitura
}
