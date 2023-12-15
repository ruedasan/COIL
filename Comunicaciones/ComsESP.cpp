#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

RF24 radio(4, 5); // Pin CE y CSN

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(0xF0F0F0F0D2LL); // Dirección de la tubería de escritura
  radio.openReadingPipe(1, 0xF0F0F0F0E1LL); // Dirección de la tubería de lectura
  radio.startListening();  // Iniciar escucha para recibir posibles respuestas
}

void loop() {
  if (radio.available()) {
    char receivedData[32] = "";
    radio.read(&receivedData, sizeof(receivedData));
    Serial.print("Recibido desde Arduino: ");
    Serial.println(receivedData);

    const char* response = "HOLA DESDE ESP32";
    radio.stopListening();  // Detener escucha para enviar
    radio.write(response, strlen(response) + 1);
    radio.startListening();  // Volver a iniciar escucha para recibir posibles respuestas
    Serial.println("Enviado: Hola desde ESP32");
  }
}



