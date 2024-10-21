#include <WiFi.h>              // Biblioteca para conexão Wi-Fi
#include <PubSubClient.h>      // Biblioteca para MQTT
#include <DHT.h>               // Biblioteca para sensores DHT
#include <Update.h>            // Biblioteca para atualizações OTA
#include <ArduinoJson.h>       // Biblioteca para manipulação de JSON
#include <ModbusMaster.h>      // Biblioteca para Modbus
#include <TinyGPSPlus.h>  // Inclui a biblioteca TinyGPSPlus


TinyGPSPlus gps;  // Cria uma instância do objeto TinyGPSPlus

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
// Definições dos pinos do NEO-6M
#define GPS_RX_PIN 16  // Pino RX do NEO-6M conectado ao TX do ESP32
#define GPS_TX_PIN 17  // Pino TX do NEO-6M conectado ao RX do ESP32

// Definições do Modbus
#define MODBUS_SERIAL_RX 16      // Pino RX do Modbus
#define MODBUS_SERIAL_TX 17      // Pino TX do Modbus

DHT dht(DHTPIN, DHTTYPE);         // Inicializa o sensor DHT

WiFiClient espClient;             // Cliente Wi-Fi
PubSubClient client(espClient);   // Cliente MQTT
ModbusMaster node;                // Instância do Modbus

void setup() {
    Serial.begin(115200);  // Inicializa a comunicação serial
    dht.begin();           // Inicializa o sensor DHT
    Serial1.begin(9600, SERIAL_8N1, MODBUS_SERIAL_RX, MODBUS_SERIAL_TX); // Configuração da Serial para Modbus
    node.begin(1, Serial1); // ID do dispositivo Modbus (substitua pelo ID correto)

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
    String url = String(firmwareUrl);
    int hostEndIndex = url.indexOf('/', 7); // Encontrar o primeiro '/' após "http://"
    String host = url.substring(7, hostEndIndex); // Host sem "http://"
    String path = url.substring(hostEndIndex); // Caminho do arquivo

    // Conectar ao servidor
    WiFiClient client;
    if (client.connect(host.c_str(), 80)) {
        Serial.println("Conectando ao servidor para atualizar o firmware...");

        // Requisição HTTP GET
        client.print(String("GET ") + path + " HTTP/1.1\r\n" +
                     "Host: " + host + "\r\n" +
                     "Connection: close\r\n\r\n");

        // Aguarda a resposta
        while (client.connected() || client.available()) {
            if (client.available()) {
                String line = client.readStringUntil('\n');
                Serial.println(line); // Imprime as linhas da resposta do servidor
                if (line.length() == 0) {
                    break; // Cabeçalho terminado
                }
            }
        }

        // Atualização
        if (Update.begin(client.available())) {
            size_t written = Update.writeStream(client);
            if (written == Update.size()) {
                Serial.println("Atualização concluída com sucesso!");
                Update.end();
                ESP.restart(); // Reinicia o ESP após a atualização
            } else {
                Serial.println("Erro durante a atualização!");
                Update.end();
            }
        } else {
            Serial.println("Falha ao iniciar a atualização!");
        }
    } else {
        Serial.println("Falha na conexão ao servidor.");
    }
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
    }
}

// Função para ler dados simulados do GY-87
void readGY87Sensors() {
    // Simulação de dados do sensor
    float simulatedPressure = 1013.25 + random(-5, 6); // Simula pressão atmosférica em hPa
    float simulatedAccelX = random(-10, 11) * 0.1;    // Simula aceleração X em g
    float simulatedAccelY = random(-10, 11) * 0.1;    // Simula aceleração Y em g
    float simulatedAccelZ = random(-10, 11) * 0.1;    // Simula aceleração Z em g
    float simulatedGyroX = random(-500, 501) * 0.1;   // Simula giroscópio X em °/s
    float simulatedGyroY = random(-500, 501) * 0.1;   // Simula giroscópio Y em °/s
    float simulatedGyroZ = random(-500, 501) * 0.1;   // Simula giroscópio Z em °/s
    float simulatedMagX = random(-100, 101) * 0.1;    // Simula magnetômetro X em μT
    float simulatedMagY = random(-100, 101) * 0.1;    // Simula magnetômetro Y em μT
    float simulatedMagZ = random(-100, 101) * 0.1;    // Simula magnetômetro Z em μT

    // Cria um objeto JSON para publicação
    DynamicJsonDocument doc(256);
    doc["frota_ID"] = FROTA_ID;  // Adiciona o ID da frota
    doc["truck_id"] = TRUCK_ID;  // Adiciona o ID do caminhão
    doc["pressure"] = simulatedPressure;
    doc["acceleration_x"] = simulatedAccelX;
    doc["acceleration_y"] = simulatedAccelY;
    doc["acceleration_z"] = simulatedAccelZ;
    doc["gyro_x"] = simulatedGyroX;
    doc["gyro_y"] = simulatedGyroY;
    doc["gyro_z"] = simulatedGyroZ;
    doc["magnetic_x"] = simulatedMagX;
    doc["magnetic_y"] = simulatedMagY;
    doc["magnetic_z"] = simulatedMagZ;

    // Serializa e publica
    String jsonString;
    serializeJson(doc, jsonString);
    client.publish("redefrigo/sensor_data", jsonString.c_str());
    
    // Imprime os dados no monitor serial
    Serial.print("Publicando no MQTT: ");
    Serial.println(jsonString);
}

// Função para ler dados simulados do sensor MQ-3
void readMQ3Sensor() {
    // Simulação de dados do sensor MQ-3
    float simulatedAlcoholLevel = random(0, 100) * 0.1; // Simula nível de álcool em mg/L

    // Cria um objeto JSON para publicação
    DynamicJsonDocument doc(200);
    doc["frota_ID"] = FROTA_ID;  // Adiciona o ID da frota
    doc["truck_id"] = TRUCK_ID;  // Adiciona o ID do caminhão
    doc["alcohol_level"] = simulatedAlcoholLevel; // Nível de álcool simulado

    // Serializa e publica
    String jsonString;
    serializeJson(doc, jsonString);
    client.publish("redefrigo/sensor_data", jsonString.c_str());
    
    // Imprime os dados no monitor serial
    Serial.print("Publicando no MQTT: ");
    Serial.println(jsonString);
}

// Função para ler dados simulados do sensor SCT013
void readSCT013Sensor() {
    // Simulação de dados do sensor SCT013
    float simulatedCurrent = random(0, 101) * 0.1; // Simula corrente (0.0 a 10.0 A)
    float simulatedVoltage = 220; // Simula Tensão em Volts
    float simulatedPower = simulatedCurrent * simulatedVoltage; // Potência em Watts

    // Cálculo do consumo em kWh (considerando um intervalo de tempo)
    float timeInHours = 1.0; // Período de medição em horas (ajuste conforme necessário)
    float energyConsumedKWh = (simulatedPower / 1000) * timeInHours; // Consumo em kWh

    // Cálculo do custo da energia
    float cost = energyConsumedKWh * KWH_COST; // Custo em centavos

    // Cria um objeto JSON para publicação
    DynamicJsonDocument doc(200);
    doc["frota_ID"] = FROTA_ID;  // Adiciona o ID da frota
    doc["truck_id"] = TRUCK_ID;   // Adiciona o ID do caminhão
    doc["current"] = simulatedCurrent; // Corrente simulada
    doc["voltage"] = simulatedVoltage;  // Tensão simulada
    doc["power"] = simulatedPower; // Potência simulada
    doc["cost"] = cost; // Custo da energia em centavos

    // Serializa e publica
    String jsonString;
    serializeJson(doc, jsonString);
    client.publish("redefrigo/sensor_data", jsonString.c_str());
    
    // Imprime os dados no monitor serial
    Serial.print("Publicando no MQTT: ");
    Serial.println(jsonString);
}

// Função para ler dados do sensor GPS NEO-6M (simulação)
void readNEO6MSensor() {
    // Simulação de dados do sensor GPS
    float latitude = -23.5505 + random(-10, 11) * 0.0001; // Simula latitude
    float longitude = -46.6333 + random(-10, 11) * 0.0001; // Simula longitude
    float altitude = 760 + random(-10, 11); // Simula altitude em metros

    // Cria um objeto JSON para publicação
    DynamicJsonDocument doc(200);
    doc["frota_ID"] = FROTA_ID;  // Adiciona o ID da frota
    doc["truck_id"] = TRUCK_ID;  // Adiciona o ID do caminhão
    doc["latitude"] = latitude;
    doc["longitude"] = longitude;
    doc["altitude"] = altitude;

    // Serializa e publica
    String jsonString;
    serializeJson(doc, jsonString);
    client.publish("redefrigo/sensor_data", jsonString.c_str());
    
    // Imprime os dados no monitor serial
    Serial.print("Publicando dados GPS no MQTT: ");
    Serial.println(jsonString);
}




// Função para ler dados do Modbus
void readModbusData() {
    uint8_t result;
    
    // Endereços a serem lidos
    uint16_t addresses[] = {256, 257, 258, 259, 260};
    float simulatedValues[5];

    // Cria um objeto JSON para publicação
    DynamicJsonDocument doc(256);
    
    for (int i = 0; i < 5; i++) {
        // Simulação dos dados
        simulatedValues[i] = random(0, 100) + random(0, 99) / 100.0; // Gera valores aleatórios de 0.00 a 100.99

        // Envia o pedido de leitura ao dispositivo Modbus
        result = node.readHoldingRegisters(addresses[i], 1); // Lê 1 registro

        if (result == node.ku8MBSuccess) {
            // Supondo que o valor recebido seja uma leitura do Modbus
            uint16_t value = node.getResponseBuffer(0);
            Serial.print("Endereço ");
            Serial.print(addresses[i]);
            Serial.print(": ");
            Serial.print(value);
            Serial.println(" (Valor Modbus)");

            // Adiciona o valor do Modbus ao objeto JSON
            switch (i) {
                case 0: doc["tensao_fotovoltaico"] = value; break;     // Tensão do módulo fotovoltaico
                case 1: doc["corrente_fotovoltaica"] = value; break;   // Corrente fotovoltaica
                case 2: doc["potencia_fotovoltaico"] = value; break;   // Potência fotovoltaica
                case 3: doc["tensao_carga_bateria"] = value; break;    // Tensão carga bateria
                case 4: doc["corrente_carga_bateria"] = value; break;  // Corrente carga bateria
            }
        } else {
            Serial.print("Falha ao ler do endereço ");
            Serial.print(addresses[i]);
            Serial.print(": ");
            Serial.println(result);
        }
    }

    // Simulando a leitura dos dados
    Serial.println("Dados simulados:");
    Serial.print("Tensão do módulo fotovoltaico: ");
    Serial.print(simulatedValues[0]); // Tensão fotovoltaica
    Serial.println(" V");
    Serial.print("Corrente fotovoltaica: ");
    Serial.print(simulatedValues[1]); // Corrente fotovoltaica
    Serial.println(" A");
    Serial.print("Potência fotovoltaica: ");
    Serial.print(simulatedValues[2]); // Potência fotovoltaica
    Serial.println(" W");
    Serial.print("Tensão carga bateria: ");
    Serial.print(simulatedValues[3]); // Tensão carga bateria
    Serial.println(" V");
    Serial.print("Corrente carga bateria: ");
    Serial.print(simulatedValues[4]); // Corrente carga bateria
    Serial.println(" A");

    // Adiciona os dados simulados ao objeto JSON
    doc["tensao_fotovoltaico_simulada"] = simulatedValues[0];
    doc["corrente_fotovoltaica_simulada"] = simulatedValues[1];
    doc["potencia_fotovoltaico_simulada"] = simulatedValues[2];
    doc["tensao_carga_bateria_simulada"] = simulatedValues[3];
    doc["corrente_carga_bateria_simulada"] = simulatedValues[4];

    // Serializa e publica os dados
    String jsonString;
    serializeJson(doc, jsonString);
    client.publish("redefrigo/sensor_data", jsonString.c_str());
    
    // Imprime os dados no monitor serial
    Serial.print("Publicando no MQTT: ");
    Serial.println(jsonString);
}

// Função para ler dados simulados do TPMS
void readTPMSSensor() {
    // Simulação de dados dos pneus
    DynamicJsonDocument doc(512); // Cria um objeto JSON

    for (int i = 0; i < 8; i++) {
        // Simula pressão (entre 30 e 40 psi) e temperatura (entre 20 e 40 °C)
        float simulatedPressure = random(30, 41) + random(0, 100) / 100.0; // Pressão em psi
        float simulatedTemperature = random(20, 41) + random(0, 100) / 100.0; // Temperatura em °C

        // Adiciona os dados ao objeto JSON
        doc["pneu_" + String(i + 1) + "_pressao"] = simulatedPressure;
        doc["pneu_" + String(i + 1) + "_temperatura"] = simulatedTemperature;

        // Imprime os dados simulados no monitor serial
        Serial.print("Pneu ");
        Serial.print(i + 1);
        Serial.print(" - Pressão: ");
        Serial.print(simulatedPressure);
        Serial.print(" psi, Temperatura: ");
        Serial.print(simulatedTemperature);
        Serial.println(" °C");
    }

    // Serializa e publica os dados
    String jsonString;
    serializeJson(doc, jsonString);
    client.publish("redefrigo/sensor_data", jsonString.c_str());

    // Imprime os dados no monitor serial
    Serial.print("Publicando dados TPMS no MQTT: ");
    Serial.println(jsonString);
}


// Callback para mensagens MQTT
void callback(char* topic, byte* payload, unsigned int length) {
    // Adiciona um terminador nulo ao payload
    payload[length] = '\0'; 

    // Verifica o tópico da mensagem recebida
    if (strcmp(topic, "redefrigo/Resp") == 0) {
        Serial.printf("Mensagem recebida no tópico %s: %s\n", topic, (const char*)payload);

        // Aqui você pode processar a mensagem recebida
        // Exemplo: Controle do TPMS, atualização de parâmetros, etc.
        
        // Se a mensagem for um comando para calibrar os pneus
        if (strcmp((const char*)payload, "calibrar") == 0) {
            // Chame a função que inicia a calibração dos pneus
            Serial.println("Iniciando calibração dos pneus...");
            // adicionar lógica de calibração aqui
        } else {
            Serial.println("Comando desconhecido.");
        }
    }
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
    // Lê e publica os dados do GPS a cada 10 segundos
    readNEO6MSensor();

    delay(10000); // Espera 10 segundos antes da próxima leitura
}
