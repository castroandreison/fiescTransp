#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define WIFI_CHAR_UUID "12345678-1234-1234-1234-123456789013"  // UUID para as credenciais Wi-Fi
#define SENSOR_CHAR_UUID "12345678-1234-1234-1234-123456789014" // UUID para os dados dos sensores

BLEServer *pServer = nullptr;
BLECharacteristic *pWiFiCharacteristic = nullptr;
BLECharacteristic *pSensorCharacteristic = nullptr;

// Função para retornar os dados do sensor (exemplo)
String getSensorData() {
    // Simulação de dados de temperatura e umidade
    String sensorData = "{\"Temperature\": 25.0, \"Humidity\": 60.0}";
    return sensorData;
}

void setup() {
    Serial.begin(115200);
    BLEDevice::init("ESP32_Test");

    pServer = BLEDevice::createServer();

    // Criar um novo serviço BLE
    BLEService *pService = pServer->createService("12345678-1234-1234-1234-123456789012");

    // Criação da característica para as credenciais Wi-Fi
    pWiFiCharacteristic = pService->createCharacteristic(
        WIFI_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );

    // Criação da característica para os dados do sensor
    pSensorCharacteristic = pService->createCharacteristic(
        SENSOR_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ
    );

    // Iniciar o serviço
    pService->start();

    // Iniciar a publicidade
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(pService->getUUID());
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Ajusta o intervalo de conexão
    BLEDevice::startAdvertising();

    Serial.println("Servidor BLE iniciado e aguardando conexões...");
}

void loop() {
    // Verifica se há dados recebidos na característica de Wi-Fi
    if (pWiFiCharacteristic->getValue().length() > 0) {
        String credentials = pWiFiCharacteristic->getValue().c_str();
        Serial.println("Credenciais recebidas: " + credentials);
        // Aqui você pode processar as credenciais, como conectar ao Wi-Fi.
        pWiFiCharacteristic->setValue(""); // Limpa o valor após leitura
    }

    // Atualiza os dados do sensor
    String sensorData = getSensorData();
    pSensorCharacteristic->setValue(sensorData.c_str());
    
    delay(2000); // Aguarda um pouco antes de atualizar os dados
}
