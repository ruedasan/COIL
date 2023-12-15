#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10); // Pin CE y CSN

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(0xF0F0F0F0E1LL); // Dirección de la tubería de escritura
  radio.openReadingPipe(1, 0xF0F0F0F0D2LL); // Dirección de la tubería de lectura
  radio.startListening();  // Iniciar escucha para recibir posibles respuestas
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'c') {
      const char* dataToSend = "Hola desde Arduino";
      radio.stopListening();  // Detener escucha para enviar
      radio.write(dataToSend, strlen(dataToSend) + 1);
      radio.startListening();  // Volver a iniciar escucha para recibir posibles respuestas
      Serial.println("Enviado: Hola desde Arduino");
    }
  }

  if (radio.available()) {
    char receivedData[32] = "";
    radio.read(&receivedData, sizeof(receivedData));
    Serial.print("Recibido desde ESP32: ");
    Serial.println(receivedData);
  }
}
