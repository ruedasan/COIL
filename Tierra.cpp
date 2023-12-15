#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);  // Configura el objeto radio en los pines CE y CSN

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(1, 0xF0F0F0F0E1LL);  // Dirección del canal de lectura (igual en el transmisor)
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    char receivedData[100];  // Ajusta el tamaño según la longitud máxima de tus datos
    radio.read(&receivedData, 100);
    Serial.print("Datos recibidos: ");
    Serial.println(receivedData);
  }
}