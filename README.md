# fiescTransp
Este projeto visa desenvolver uma solução robusta e integrada para monitoramento em tempo real de um caminhão, utilizando uma variedade de sensores e tecnologias de comunicação para coleta, armazenamento e transmissão de dados críticos sobre o veículo.

A arquitetura do sistema é construída em torno da placa ESP32, que se conecta à rede Wi-Fi para enviar dados via MQTT a um servidor remoto, onde as informações podem ser processadas e analisadas. O sistema utiliza sensores como DHT para medir temperatura e umidade, GY-87 para dados de aceleração e giroscópio, MQ-3 para detectar níveis de álcool, e SCT013 para monitoramento de corrente elétrica. Além disso, há uma interface Modbus para leitura de dados de sensores fotovoltaicos e de bateria, e um sistema de monitoramento de pressão e temperatura dos pneus (TPMS).

Os dados são capturados periodicamente e publicados no broker MQTT para visualização e armazenamento, permitindo que gestores monitorem remotamente as condições do veículo. Adicionalmente, o código integra funcionalidades para atualizações de firmware OTA, o que facilita a manutenção e atualização do software sem a necessidade de acesso físico ao dispositivo.

Este projeto também inclui medidas de segurança, como a conexão segura via MQTT e a potencial implementação de certificados de IoT, além de prever uma interface via BLE para configuração inicial do dispositivo, proporcionando flexibilidade no gerenciamento da conectividade do caminhão.

Por fim, o sistema prevê que todos os dados coletados sejam gravados em um cartão SD, garantindo um backup local das informações caso a conexão com o servidor seja interrompida.

