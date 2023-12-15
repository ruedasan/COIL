#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <QMC5883LCompass.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <RF24.h>


Adafruit_BMP280 bmp;  // BMP280 sensor (I2C)
TinyGPSPlus gps;       // Object to handle GPS
#define RXD2 16
#define TXD2 17
HardwareSerial neogps(1);  // Communication with NEO-6M GPS module
QMC5883LCompass compass;

Adafruit_MPU6050 mpu;

int calibrationData[3][2];
bool changed = false;
bool done = false;
int t = 0;
int c = 0;

bool calibrated = false;
int hourBogota;

String determineCardinalDirection(float azimuth);

RF24 radio(4, 5);

void printData();

void setup() {
  Serial.begin(115200);  // Serial communication with the Arduino IDE monitor
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);  // GPS module configuration
  unsigned status = bmp.begin(0x76);  // BMP280 initialization
  compass.init();  // Compass initialization
    Wire.begin();

    radio.begin();

  Serial.println("Adafruit MPU6050 test!");
  
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);
}

void loop() {
  boolean newData = false;

  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (neogps.available()) {
      if (gps.encode(neogps.read())) {
        newData = true;
      }
    }
  }

  if (!calibrated){
        //If not calibrated
      int x, y, z;
 
      //Read compass values
      compass.read();
 
     // Return XYZ readings
      x = compass.getX();
      y = compass.getY();
      z = compass.getZ();
 
      changed = false;
 
      if(x < calibrationData[0][0]) {
        calibrationData[0][0] = x;
        changed = true;
      }
      if(x > calibrationData[0][1]) {
        calibrationData[0][1] = x;
        changed = true;
      }
 
      if(y < calibrationData[1][0]) {
        calibrationData[1][0] = y;
        changed = true;
      }
      if(y > calibrationData[1][1]) {
        calibrationData[1][1] = y;
        changed = true;
      }
 
      if(z < calibrationData[2][0]) {
        calibrationData[2][0] = z;
        changed = true;
      }
      if(z > calibrationData[2][1]) {
        calibrationData[2][1] = z;
        changed = true;
      }
 
      if (changed && !done) {
        Serial.println("CALIBRATING... Keep moving your sensor around.");
        c = millis();
      }
        t = millis();
 
 
      if ( (t - c > 5000) && !done) {
        done = true;
        Serial.println("DONE.");
        Serial.println();
 
        Serial.print("compass.setCalibration(");
        Serial.print(calibrationData[0][0]);
        Serial.print(", ");
        Serial.print(calibrationData[0][1]);
        Serial.print(", ");
        Serial.print(calibrationData[1][0]);
        Serial.print(", ");
        Serial.print(calibrationData[1][1]);
        Serial.print(", ");
        Serial.print(calibrationData[2][0]);
        Serial.print(", ");
        Serial.print(calibrationData[2][1]);
        Serial.println(");");
 
        compass.setCalibration( calibrationData[0][0], calibrationData[0][1], calibrationData[1][0],
                                calibrationData[1][1], calibrationData[2][0], calibrationData[2][1]);
        calibrated = true;
        }
    }

    delay(250);
    if (newData) {
    newData = false;
    //printData(); // Imprimir datos del GPS
  } else {
    Serial.println("No GPS Data");
  }
  hourBogota = gps.time.hour() - 5;
    if (hourBogota < 0) {
      hourBogota += 24;  // Manejar casos donde la resta resulta en un valor negativo
    }
printData(); // Imprimir datos del GPS
  }




void printData() {
  // if (gps.location.isValid()) {
  //   Serial.print("Time: ");
  //   Serial.print(hourBogota);
  //   Serial.print(":");
  //   Serial.print(gps.time.minute());
  //   Serial.print(":");
  //   Serial.println(gps.time.second());
  //   Serial.println("GPS Data:");
  //   Serial.print("Latitude: ");
  //   Serial.println(gps.location.lat(), 6);

  //   Serial.print("Longitude: ");
  //   Serial.println(gps.location.lng(), 6);
  // } else {
  //   Serial.println("Invalid GPS Data");
  // }

  // Serial.println("BMP280 Data:");
  // Serial.print(F("Temperature = "));
  // Serial.print(bmp.readTemperature());
  // Serial.println(" *C");

  // Serial.print(F("Pressure = "));
  // Serial.print(bmp.readPressure());
  // Serial.println(" Pa");

  // Serial.print(F("Approx altitude = "));
  // Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local prediction */
  // Serial.println(" m");

     int x, y, z;

  // //Read compass values
  // compass.read();

  // //Return XYZ readings
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  // int azimut = compass.getAzimuth();


  // //Write direction
  // //Reemplaza las condiciones existentes para todas las direcciones con las siguientes
  // azimut = (azimut + 360) % 360; // Asegura que azimut esté en el rango de 0 a 360 grados
  // Serial.println("GY-273 Data:");

  // if ((azimut >= 337.5) || (azimut < 22.5))
  //   Serial.print("North     ");
  // if ((azimut >= 22.5) && (azimut < 67.5))
  //   Serial.print("North-East");
  // if ((azimut >= 67.5) && (azimut < 112.5))
  //   Serial.print("East      ");
  // if ((azimut >= 112.5) && (azimut < 157.5))
  //   Serial.print("South-East");
  // if ((azimut >= 157.5) && (azimut < 202.5))
  //   Serial.print("South     ");
  // if ((azimut >= 202.5) && (azimut < 247.5))
  //   Serial.print("South-West");
  // if ((azimut >= 247.5) && (azimut < 292.5))
  //   Serial.print("West      ");
  // if ((azimut >= 292.5) && (azimut < 337.5))
  //   Serial.print("North-West");

  // Serial.print(" Azimuth: ");
  // Serial.print(azimut);

  // /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // /* Print out the values */
  // Serial.print("Acceleration X: ");
  // Serial.print(a.acceleration.x);
  // Serial.print(", Y: ");
  // Serial.print(a.acceleration.y);
  // Serial.print(", Z: ");
  // Serial.print(a.acceleration.z);
  // Serial.println(" m/s^2");

  // Serial.print("Rotation X: ");
  // Serial.print(g.gyro.x);
  // Serial.print(", Y: ");
  // Serial.print(g.gyro.y);
  // Serial.print(", Z: ");
  // Serial.print(g.gyro.z);
  // Serial.println(" rad/s");


   // Obtener la hora actual
  String hora = String(hourBogota) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());

  // Obtener la posición cardinal
  String direccion;
  int azimut = compass.getAzimuth();
  azimut = (azimut + 360) % 360;

  if ((azimut >= 337.5) || (azimut < 22.5))
    direccion = "North";
  else if ((azimut >= 22.5) && (azimut < 67.5))
    direccion = "North-East";
  else if ((azimut >= 67.5) && (azimut < 112.5))
    direccion = "East";
  else if ((azimut >= 112.5) && (azimut < 157.5))
    direccion = "South-East";
  else if ((azimut >= 157.5) && (azimut < 202.5))
    direccion = "South";
  else if ((azimut >= 202.5) && (azimut < 247.5))
    direccion = "South-West";
  else if ((azimut >= 247.5) && (azimut < 292.5))
    direccion = "West";
  else if ((azimut >= 292.5) && (azimut < 337.5))
    direccion = "North-West";

  // Obtener el ángulo de giro
  String aceleracion = "X=" + String(a.acceleration.x) + " Y=" + String(a.acceleration.y) + " Z=" + String(a.acceleration.z)+ "(mts/seg)^2";

  // Obtener otros valores
  String latitud = String(gps.location.lat(), 6);
  String longitud = String(gps.location.lng(), 6);
  String altura = String(bmp.readAltitude(1013.25)) + "mts";  // Ajusta según predicción local
  String temperatura = String(bmp.readTemperature()) + " ºC";
  String presion = String(bmp.readPressure()/100) + " Pa";
  String giro = "Gyro X=" + String(g.gyro.x) + " Y=" + String(g.gyro.y) + " Z=" + String(g.gyro.z) + "rad/seg";
  String Flujos = "FlujoCampoX: " + String(x) + "uT" + " FlujoCampoY: " + String(y) + "uT" + " FlujoCampoZ: " + String(z) + "uT";


String datos = "H:" + hora + ", D:" + direccion + ", A:" + aceleracion + ", La:" + latitud + ", Lo:" + longitud + ", " +
                  "Al:" + altura + ", Te:" + temperatura + ", Pr:" + presion + ", Gi:" + giro;
  Serial.println(datos);
  // Ahora puedes hacer lo que quieras con la cadena 'datos'
String datos1 = "H:" + hora;
String datos2 = "D:" + direccion;
String datos3 = "A:" + aceleracion;
String datos4 = "La:" + latitud;
String datos5 = "Lo:" + longitud;
String datos6 = "Al:" + altura;
String datos7 = "Te:" + temperatura;
String datos8 = "Pr:" + presion;
String datos9 = "Gi:" + giro;
String datos10 = "Flujo" + Flujos;

Serial.println(datos1);
Serial.println(datos2);
Serial.println(datos3);
Serial.println(datos4);
Serial.println(datos5);
Serial.println(datos6);
Serial.println(datos7);
Serial.println(datos8);
Serial.println(datos9);
Serial.println(datos10);


  char charArray1[datos1.length() + 1];
  char charArray2[datos2.length() + 1];
  char charArray3[datos3.length() + 1];
  char charArray4[datos4.length() + 1];
  char charArray5[datos5.length() + 1];
  char charArray6[datos6.length() + 1];
  char charArray7[datos7.length() + 1];
  char charArray8[datos8.length() + 1];
  char charArray9[datos9.length() + 1];

  datos1.toCharArray(charArray1, sizeof(charArray1));
  datos2.toCharArray(charArray2, sizeof(charArray2));
  datos3.toCharArray(charArray3, sizeof(charArray3));
  datos4.toCharArray(charArray4, sizeof(charArray4));
  datos5.toCharArray(charArray5, sizeof(charArray5));
  datos6.toCharArray(charArray6, sizeof(charArray6));
  datos7.toCharArray(charArray7, sizeof(charArray7));
  datos8.toCharArray(charArray8, sizeof(charArray8));
  datos9.toCharArray(charArray9, sizeof(charArray9));

  // Enviar la cadena a través del módulo NRF24L01
  radio.openWritingPipe(0xF0F0F0F0E1LL);  // Dirección del canal de escritura (igual en el receptor)
  radio.write(charArray1, sizeof(charArray1));
  radio.write(charArray2, sizeof(charArray2));
  radio.write(charArray3, sizeof(charArray3));
  radio.write(charArray4, sizeof(charArray4));
  radio.write(charArray5, sizeof(charArray5));
  radio.write(charArray6, sizeof(charArray6));
  radio.write(charArray7, sizeof(charArray7));
  radio.write(charArray8, sizeof(charArray8));
  radio.write(charArray9, sizeof(charArray9));


}



