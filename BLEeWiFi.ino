#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// UUIDs do serviço e características BLE
#define SERVICE_UUID "12345678-1234-1234-1234-123456789012"
#define WIFI_CHAR_UUID "12345678-1234-1234-1234-123456789013"
#define SENSOR_CHAR_UUID "12345678-1234-1234-1234-123456789014"

// Variáveis para armazenar as credenciais Wi-Fi
char ssid[32];
char password[32];
char company[32];
char plate[32];

// Classe do servidor BLE
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        Serial.println("Dispositivo BLE conectado.");
    }

    void onDisconnect(BLEServer* pServer) {
        Serial.println("Dispositivo BLE desconectado.");
    }
};

// Função para inicializar o BLE
void initBLE() {
    BLEDevice::init("ESP32_Sensor"); // Nome do dispositivo BLE
    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    // Cria um serviço BLE
    BLEService* pService = pServer->createService(SERVICE_UUID);

    // Cria uma característica para as credenciais Wi-Fi
    BLECharacteristic* pWifiChar = pService->createCharacteristic(
        WIFI_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    
    // Cria uma característica para enviar dados do sensor
    BLECharacteristic* pSensorChar = pService->createCharacteristic(
        SENSOR_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pSensorChar->addDescriptor(new BLE2902());

    // Inicia o serviço BLE
    pService->start();
    BLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
    Serial.println("Aguardando conexão BLE...");
}

// Função para lidar com a escrita da característica de Wi-Fi
void wifiCharWritten(BLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    Serial.println("Recebendo credenciais de Wi-Fi...");

    // Parse JSON
    DynamicJsonDocument doc(256);
    deserializeJson(doc, value);

    // Armazena as credenciais
    strlcpy(ssid, doc["SSID"], sizeof(ssid));
    strlcpy(password, doc["Password"], sizeof(password));
    strlcpy(company, doc["Company"], sizeof(company));
    strlcpy(plate, doc["Plate"], sizeof(plate));

    Serial.printf("SSID: %s\nPassword: %s\nCompany: %s\nPlate: %s\n", ssid, password, company, plate);

    // Conectar ao Wi-Fi
    WiFi.begin(ssid, password);
    Serial.println("Conectando ao Wi-Fi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConectado ao Wi-Fi.");
}

// Função para enviar dados do sensor
void sendSensorData(BLECharacteristic* pSensorChar) {
    // Simulação de dados do sensor
    float temperature = random(20, 30); // Temperatura simulada
    float humidity = random(40, 60);    // Umidade simulada

    // Cria um objeto JSON para os dados do sensor
    DynamicJsonDocument doc(128);
    doc["Temperature"] = temperature;
    doc["Humidity"] = humidity;

    // Serializa e envia
    String jsonString;
    serializeJson(doc, jsonString);
    pSensorChar->setValue(jsonString.c_str());
    pSensorChar->notify();
    Serial.printf("Dados do sensor enviados: %s\n", jsonString.c_str());
}

void setup() {
    Serial.begin(115200);
    initBLE();

    // Obter a característica do sensor
    BLECharacteristic* pSensorChar = BLEDevice::createServer()->getCharacteristic(SENSOR_CHAR_UUID);
    // Define a função que será chamada quando o WifiChar for escrita
    BLEDevice::createServer()->getCharacteristic(WIFI_CHAR_UUID)->setCallbacks(new MyServerCallbacks(wifiCharWritten));

    // Loop para enviar dados do sensor a cada 5 segundos
    while (true) {
        sendSensorData(pSensorChar);
        delay(5000); // Envia a cada 5 segundos
    }
}

void loop() {
    // O loop principal está vazio, pois todas as ações ocorrem nos callbacks
}
