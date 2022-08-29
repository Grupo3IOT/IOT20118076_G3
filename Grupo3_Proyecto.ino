/***INCLUIR LIBRERIAS***/
#include <WiFi.h> //Libreria para el WiFi
#include <PubSubClient.h> //Libreria para la comunicacion MQTT
#include "DHTesp.h" //libreria del sensor DHT22
#include <LiquidCrystal_I2C.h> //Libreria para usar LCD I2C
/**PINES**/
//Sensores
int pindht22 = 16; //pin G16 conectado al sensor DHT22
int pinsensorHS = 35; //pin G35 conectado al sensor de humedad de suelo (FC-28)
//Rele
int rele = 15; //pin G15 conectado al rele
//LCD I2C
int pinSDA = 21; //pin G21 conectado al sistema de data del LCD I2C
int pinSCL = 22; //pin G22 conectado al sistema de reloj del LCD I2C

//Sensor de distancia HC-SR04
int trigPin = 12;   //pin G12 conectado al trigger del sensor
int echoPin = 14;   //pin G14 conectado al echo del sensor

//LED RGB
int pinRed = 17; //pin G17 conectado al anodo rojo
int pinGreen = 18; //pin G18 conectado al anodo verde
int pinBlue = 5; //pin G5 conectado al anodo azul

//LED Rojo
int ledrojo = 13; //pin G13 conectado al LED Rojo

//LED multicolor
int ledMulti = 33; //pin G33 conectado al LED multicolor

/***VARIABLES GLOBALES***/
/*  Crea un objeto de la clase LiquidCrystal_I2C
    con dirección, columnas y filas indicadas.*/
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHTesp dhtSensor; //variable que representa al sensor DHT22
WiFiClient esp32Client; //Instancia de cliente para WiFi
PubSubClient mqttClient(esp32Client); //Instancia de cliente para MQTT

/***DATOS DE LA RED WIFI***/
const char* ssid = "HUAWEI-2.4G-YpsU"; //Nombre del WiFi
const char* password = "5F32kT8q"; //Contrasenia del WiFi

/***DATOS DEL SERVIDOR BROKER MQTT***/
char *server = "192.168.18.60"; //IP del servidor
int port = 1883; //Puerto para la conexion
String resultS = ""; //Variable que representa el mensaje

/********************************************************/

/***PROTOTIPOS DE FUNCIONES***/
void wifiInit();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
long readUltrasonicDistance(int triggerPin, int echoPin);
double sensorDistancia();
void sensorSuelo(double HS, double pcHS);
void sensorHumedadTemperatura(double humedad, double temp);
void linea();
void imprimirLCD(double humedad, double temperatura, double hs, double pcHS);
bool nivelAgua();
int tierraSeca(double hs);
void MQQT_conexion(double h, double t, double hs, double p_hs);

/********************************************************/
/***SETUP***/
//Inicializar los sensores y configuraciones
void setup()
{
  Serial.begin(9600); //Inicializa la consola

  wifiInit(); //Se inicializa el WiFi
  //Establece los detalles del servidor: IP del servidor y puerto de conexion
  mqttClient.setServer(server, port);
  //Establece el procedimiento para la devolucion de llamada de mensaje
  mqttClient.setCallback(callback);

  //Inicializar los pines
  pinMode(rele, OUTPUT); //pin G15 como salida
  pinMode(pinsensorHS, INPUT); //pin G35 como entrada
  pinMode(pinRed, OUTPUT); //pin G17 como salida
  pinMode(pinGreen, OUTPUT); //pin G18 como salida
  pinMode(pinBlue, OUTPUT); //pin G5 como salida
  pinMode(ledrojo, OUTPUT); //pin G13 como salida
  pinMode(ledMulti, OUTPUT); //pin G33 como salida

  //Inicializar el sensor DHT22 especificando el pin G16 y el tipo de sensor
  dhtSensor.setup(pindht22, DHTesp::DHT22);
  //Funciones para configurar el LCD I2C
  lcd.init();
  lcd.backlight();
}
/********************************************************/
/***LOOP***/
//Ejecuta el codigo repetitivamente
void loop()
{

  TempAndHumidity  data = dhtSensor.getTempAndHumidity(); //Captura valores del sensor DHT22
  double sensorHS = analogRead(pinsensorHS); //Captura valor del sensor de humedad del suelo (sensorHS)
  double HS = map(sensorHS, 0, 4096, 0, 1023); //Mapea el valor de sensorHS en un rango de 0 a 1023
  double pcHS = map(sensorHS, 0, 4096, 100, 0); //Mapea el valor de sensorHS en un rango de 100% a 0%
  int humedadTierra = tierraSeca(HS); //Asigna el estado del agua (seca/humeda/mojada)

  MQQT_conexion(data.humidity, data.temperature, HS, pcHS); //Conecta a MQTT
  imprimirLCD(data.humidity, data.temperature, HS, pcHS); //Imprime valores en el LCD

  if (nivelAgua()) { //Si hay agua suficiente
    if (humedadTierra == 3) { //Si la tierra esta seca
      //Encender el color azul del LED RGB
      digitalWrite(pinRed, 0);
      digitalWrite(pinGreen, 0);
      digitalWrite(pinBlue, 255);
      //Apagar el LED rojo
      digitalWrite(ledrojo, LOW);
      digitalWrite(rele, LOW); // Enviar un valor bajo al rele y enciende la bomba de agua
      lcd.clear(); //Limpia el LCD
      lcd.setCursor(0, 0); //Se ubica el curso en las posiciones (columna:0, fila:0)
      lcd.print("Regando..."); //Imprime la frase en el LCD
    } 
    else { //Sino, no esta seca
      if (humedadTierra == 2) { //Si la tierra esta humeda
        if (data.temperature > 23) { //Si la temperatura del ambiente es mayor a 23°C
          //Encender el color azul del LED RGB
          digitalWrite(pinRed, 0);
          digitalWrite(pinGreen, 0);
          digitalWrite(pinBlue, 255);
          //Apagar el LED rojo
          digitalWrite(ledrojo, LOW);
          digitalWrite(rele, LOW); // Enviar un valor bajo al rele y enciende la bomba de agua
          lcd.clear(); //Limpia el LCD
          lcd.setCursor(0, 0); //Se ubica el curso en las posiciones (columna:0, fila:0)
          lcd.print("Regando..."); //Imprime la frase en el LCD
        } 
        else { //Sino, la temperatura del ambiente es menor que 23°C
          //Encender el color verde del LED RGB
          digitalWrite(pinRed, 0);
          digitalWrite(pinGreen, 255);
          digitalWrite(pinBlue, 0);
          //Apagar el LED rojo
          digitalWrite(ledrojo, LOW);
          digitalWrite(rele, HIGH); //Envia un valor alto al rele y apaga la bomba de agua
        }
      } 
      else { //Sino, la tierra esta mojada
        //Encender el color verde del LED RGB
        digitalWrite(pinRed, 0);
        digitalWrite(pinGreen, 255);
        digitalWrite(pinBlue, 0);
        //Apagar el LED rojo
        digitalWrite(ledrojo, LOW);
        digitalWrite(rele, HIGH); //Envia un valor alto al rele y apaga la bomba de agua
      }
    }
  } 
  else { //Sino, no hay agua suficiente o el tanque esta vacío
    digitalWrite(ledrojo, HIGH); //Encender el LED rojo
    //Apagar LED RGB
    digitalWrite(pinRed, 0);
    digitalWrite(pinGreen, 0);
    digitalWrite(pinBlue, 0);
    //-----------------------
    digitalWrite(rele, HIGH); //Envia un valor alto al rele y apaga la bomba de agua
    //Imprimir en el LCD
    lcd.clear(); //Limpia el LCD
    lcd.setCursor(0, 0); //Se ubica el curso en las posiciones (columna:0, fila:0)
    lcd.print("No hay agua,"); //Imprime la frase en el LCD
    lcd.setCursor(0, 1); //Se ubica el curso en las posiciones (columna:0, fila:1)
    lcd.print("rellene de agua"); //Imprime la frase en el LCD
  }

  if (data.humidity >= 50 && data.humidity <= 75) { //Si la humedad detectada por el sensor DHT22 esta entre 50°C y 75°C
    digitalWrite(ledMulti, LOW); //Apagar el LED multicolor
  } 
  else { //Si no cumple la condicion anterior
    delay(2000);   //Espera por 2000 milisegundos = 2 segundos
    digitalWrite(ledMulti, HIGH); //Enceder el LED multicolor
    if (data.humidity < 50) {
      //Imprimir en el LCD
      lcd.clear(); //Limpia el LCD
      lcd.setCursor(0, 0); //Se ubica el curso en las posiciones (columna:0, fila:0)
      lcd.print("Planta en peligro");   //Imprime la frase en el LCD
      lcd.setCursor(0, 1); //Se ubica el curso en las posiciones (columna:0, fila:1)
      lcd.print("poca humedad");  //Imprime la frase en el LCD
    }
    else {
      //Imprimir en el LCD
      lcd.clear(); //Limpia el LCD
      lcd.setCursor(0, 0); //Se ubica el curso en las posiciones (columna:0, fila:0)
      lcd.print("Planta en peligro");   //Imprime la frase en el LCD
      lcd.setCursor(0, 1); //Se ubica el curso en las posiciones (columna:0, fila:1)
      lcd.print("mucha humedad");  //Imprime la frase en el LCD
    }
  }   
  delay(2000);   //Espera por 2000 milisegundos = 2 segundos
}
/********************************************************/
/***IMPLEMENTACION DE FUNCIONES***/
//Procedimiento para inicializar la configuracion del WiFi
void wifiInit() {

  Serial.print("Conectándose a "); //Imprimir en consola la cadena de caracteres enviada
  Serial.println(ssid);  //Imprimir el nombre del WiFi en consola

  WiFi.begin(ssid, password); //Inicializa la configuracion de red de la libreria WiFi tomando en cuenta el nombre y la contrasenia del WiFi

  //Bucle para conectar el WiFi
  while (WiFi.status() != WL_CONNECTED) { //Continua hasta que se haya conectado al WiFi
    Serial.print("."); //Imprimir “.” en la consola
    delay(500);  //Retrasa el programa por 500 milisegundos
  }
  //Imprimir en consola
  Serial.println("");
  Serial.println("Conectado a WiFi");
  Serial.println("Dirección IP: ");
  Serial.println(WiFi.localIP()); //Imprime la direccion IP en consola
}

//Procedimiento que maneja los mensajes recibidos
void callback(char* topic, byte* payload, unsigned int length) {
  //Imprime en consola
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("] ");

  char payload_string[length + 1];

  memcpy(payload_string, payload, length); //Concatenar cadenas de caracteres
  payload_string[length] = '\0'; //El ultimo caracter es el final de cadena
  resultS = "";
  //Bucle para unir los caracteres
  for (int i = 0; i < length; i++) {
    resultS = resultS + (char)payload[i]; //Se concatena los caracteres en resultS, es decir el mensaje recibido
  }
  Serial.println(); //Hace un salto de linea
}

//Procedimiento para reconectarse al MQTT en caso sea necesario
void reconnect() {
  while (!mqttClient.connected()) { //Verifica que no este conectado
    Serial.println("Intentando conectarse MQTT..."); //Imprime en consola
    String client_id = "esp32-client-"; //ID de cliente de esp32
    /*Se obtiene la direccion MAC del WiFi,
      se hace un cast para que sea de tipo String
      y se le concatena a la variable client_id.*/
    client_id += String(WiFi.macAddress());
    //Conectar al cliente usando su respectivo ID
    if (mqttClient.connect(client_id.c_str())) {
      Serial.println("Conectado"); //Imprime en consola
      mqttClient.subscribe("test"); //Suscribir los mensajes publicados a “test”
    } else {//Si no se conecta
      Serial.print("Fallo, rc=");
      Serial.print(mqttClient.state());  //Imprime el estado actual del cliente MQTT
      Serial.println(" intentar de nuevo en 5 segundos");
      delay(5000); // Espera por 5000 milisegundos = 5 segundos
    }
  }
}

//Retornar la onda de sonido por el sensor HC-SR04
long readUltrasonicDistance(int triggerPin, int echoPin)
{

  pinMode(triggerPin, OUTPUT); //Configura el pin G12 como salida
  digitalWrite(triggerPin, LOW); //Determina el pin G12 con un valor bajo
  delayMicroseconds(2); //Retrasa el programa por 2 microsegundos
  digitalWrite(triggerPin, HIGH); //Determina el pin G12 con un valor alto
  delayMicroseconds(10); //Retrasa el programa por 10 microsegundos
  digitalWrite(triggerPin, LOW); //Determina el pin G12 con un valor bajo
  pinMode(echoPin, INPUT); //Configura el pin G14 como entrada
  // Lee el pin echo y retorna la onda de sonido en microsegundos
  return pulseIn(echoPin, HIGH);
}

//Retorna le distancia segun el sensor HC-SR04
double sensorDistancia() {
  double cm = 0;
  //Obtiene la distancia en cm usando la onda de sonido del HC-SR04
  cm = 0.01723 * readUltrasonicDistance(trigPin, echoPin);
  Serial.println("Distancia: " + String(cm) + "cm");
  return cm;
}

//Mostrar valores del sensor de humedad del suelo en LCD y el monitor del IDE
void sensorSuelo(double HS, double pcHS) {
  //LCD
  lcd.setCursor(0, 0); //Se ubica el cursor en las posiciones (columna:0, fila:0)
  lcd.print("H. Suelo: "); //Imprime un string
  lcd.print(String(HS, 3)); //Convertir HS a string con 3 decimales y lo imprime
  lcd.print(" u"); //Imprime un string
  lcd.setCursor(0, 1); //Se ubica el cursor en las posiciones (columna:0, fila:1)
  lcd.print("H.S.(%)"); //Imprime un string
  lcd.print(String(pcHS)); //Convertir pcHS a string  y lo imprime
  lcd.print(" %"); //Imprime un string
  //MONITOR SERIAL
  Serial.println("Humedad Suelo : " + String(HS, 3) + " u "); //Imprime un string
  Serial.println("Humedad Suelo en % : " + String(pcHS) + " % "); //Imprime un string
}

//Mostrar valores segun el sensor DHT22
void sensorHumedadTemperatura(double humedad, double temp) {

  //LCD
  lcd.print("Temper.: ");
  lcd.print(String(temp, 2)); //Convierte temp a string con 2 decimales y lo imprime
  lcd.print(" Celcius");
  lcd.setCursor(0, 1); //Se ubica el curso en las posiciones (columna:0, fila:1)
  lcd.print("Humedad: ");
  lcd.print(String(humedad)); //Convertir humedad a string  y lo imprime
  lcd.print("%");

  //MONITOR SERIAL
  Serial.println("Temperatura: " + String(temp, 2) + " Celsius "); //Imprime valores
  Serial.println("Humedad Aire: " + String(humedad) + " % "); //Imprime valores
}

//Imprime una linea de dobles rayas en el monitor
void linea() {
  Serial.println("==========================");
}

//Imprimer los valores en el LCD de forma general
void imprimirLCD(double humedad, double temperatura, double hs, double pcHS) {
  lcd.clear();  //Limpiar la pantalla del LCD
  sensorHumedadTemperatura(humedad, temperatura); //Imprime los valores del DHT22
  delay(1000); //Pausa el programa por 1 segundo

  lcd.clear(); //Limpiar la pantalla del LCD
  sensorSuelo(hs, pcHS); //Imprime el valor del sensor de humedad del suelo
  delay(1000); //Pausa el programa por 1 segundo

  linea(); //Imprime la linea en el monitor
}

//Determina el nivel del agua en el tanque
bool nivelAgua() {
  //Variables
  double distancia = 0;
  bool agua = false; //La cantidad de agua no es suficiente o ya no hay
  distancia = sensorDistancia(); //Asigna la disntancia por el HC-SR04

  if (distancia < 15.5) { //Si la distancia es menor que 15.5 cm
    agua = true; //La cantidad de agua es suficiente
  }

  return agua; //Retorna true o false
}

//Determina si la tierra esta seca/humeda/mojada
int tierraSeca(double hs) {
  //Se realiza de acuerdo al valor obtenido de forma analogica del sensor de humedad del suelo
  int estado;
  if (hs <= 300) {
    estado = 1; // Tierra mojada
  } else {
    if (hs <= 700) {
      estado = 2; // Tierra humeda
    } else {
      estado = 3; // Tierra seca
    }
  }
  return estado; //Retorna un numero entero de acuerdo al estado de la tierra
}

//Procedimiento de conexion al MQTT
void MQQT_conexion(double h, double t, double hs, double p_hs) {
  if (!mqttClient.connected()) { //Verifica si el cliente para MQTT no esta conectado
    reconnect(); //Reconectar en caso no este conectado
  }

  mqttClient.loop(); //Permite que el cliente procese los mensajes entrantes y mantenga la conexion al servidor

  /*Sensor DHT22*/
  // Se pasa el valor de la temperatura de DHT22 al servidor MQTT
  mqttClient.publish("esp32/Temperatura", String(t).c_str());

  //Se pasa el valor de la humedad del aire de DHT22 al servidor MQTT
  mqttClient.publish("esp32/HumedadAire", String(h).c_str());

  /*Sensor Humedad de Tierra*/
  //Se pasa el valor del sensor de humedad del suelo al servidor MQTT
  mqttClient.publish("esp32/HumedadTierra", String(hs).c_str());

  //Se pasa el valor de la sensor de humedad del suelo en formato de % al servidor MQTT
  mqttClient.publish("esp32/HumedadTierraPorcentaje", String(p_hs).c_str());

  delay(2000); //Espera por 2000 milisegundos = 2 segundos
}
