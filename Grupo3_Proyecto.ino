
#include "DHTesp.h" //libreria del sensor DHT22
#include <Wire.h>
#include <LiquidCrystal_I2C.h>//Libreria del lcd
LiquidCrystal_I2C lcd(0x27, 16, 2); //dimension del lcd

int bomba = 15; //pin de led rojo B15 
int pindht22 = 16; //pin de sensor DHT22 B16
int pinsensorHS = 35; //pin de sensor de humedad de suelo G35
int pinSDA = 21; //pin de LCD I2C   morado
int pinSCL = 22; //pin de LCD I2C   blanco
int ledMulti = 33;

// for Arduino microcontroller
int trigPin = 12;      // trigger pin
int echoPin = 14;      // echo pin

//RGB
int pinRed = 17;
int pinGreen = 18;
int pinBlue = 5;
int ledrojo = 13;



DHTesp dhtSensor;


void setup() {
  // Bloque de codigo para ejecutar una vez
  Serial.begin(9600);  
  Serial.println("Hello, ESP32!");  
  
 // pinMode(RELE,OUTPUT);
  pinMode(bomba, OUTPUT); //inicializamos el pin de led como salida
  pinMode(pinsensorHS, INPUT);
  pinMode(pinRed,OUTPUT);
  pinMode(pinGreen,OUTPUT);
  pinMode(pinBlue,OUTPUT);
  pinMode(ledrojo,OUTPUT);
  pinMode(ledMulti,OUTPUT);
  
  dhtSensor.setup(pindht22, DHTesp::DHT22);
  //Funciones para configurar el LCD I2C
  lcd.init();
  //lcd.begin();
  lcd.backlight();    
 // lcd.setBacklight(HIGH);
  lcd.print("Hello, ESP32! ");
  
  
}





long readUltrasonicDistance(int triggerPin, int echoPin) 
{ 
 pinMode(triggerPin, OUTPUT);  
 digitalWrite(triggerPin, LOW); 
 delayMicroseconds(2); 
 digitalWrite(triggerPin, HIGH); 
 delayMicroseconds(10); 
 digitalWrite(triggerPin, LOW); 
 pinMode(echoPin, INPUT); 
 // Lee el pin echo y retorna la onda de sonido en microsegundos 
 return pulseIn(echoPin, HIGH); 
} 



void sensorSuelo(double HS, double pcHS){
  
  lcd.setCursor(0,0);
  lcd.print("H. Suelo: ");  
  lcd.print(String(HS,3));     
  lcd.print(" u");
  lcd.setCursor(0,1);
  lcd.print("H.S.(%)");  
  lcd.print(String(pcHS));     
  lcd.print(" %");

  Serial.println("Humedad Suelo : "+String(HS,3)+ " u ");
  Serial.println("Humedad Suelo en % : "+String(pcHS)+ " % ");
  
}

void sensorHumedadTemperatura(double humedad,double temp){

  lcd.print("Temper.: ");  
  lcd.print(String(temp, 2));     
  lcd.print(" Celcius");
  lcd.setCursor(0,1);
  lcd.print("Humedad: ");  
  lcd.print(String(humedad));     
  lcd.print("%");
  
}


double sensorDistancia(){
  double cm = 0; 
  cm = 0.01723 * readUltrasonicDistance(trigPin,echoPin);
  Serial.println("Distancia: " + String(cm) + "cm");
  return cm;
} 

void linea(){
  Serial.println("=========================="); 
}


void imprimirLCD(double humedad, double temperatura,double hs,double pcHS){
  lcd.clear();  
  sensorHumedadTemperatura(humedad,temperatura); 
  delay(1000);
  
  lcd.clear();
  sensorSuelo(hs,pcHS);
  delay(1000);

  linea();
}

bool nivelAgua(){
  double distancia = 0;
  bool agua = false;
  
  distancia = sensorDistancia();
  if(distancia<15.5){
      agua = true;
  }
  
  return agua;
}

int tierraSeca(double hs){
  int estado;
  if(hs<=300){
     estado = 1; // tierra mojada   
  }else{
    if(hs <=700){
      estado = 2; // tierra humeda
    }else{
      estado = 3; // tierra seca
    }  
  }
  return estado;
}


void loop() {
  //double distancia = 0;
  //distancia = sensorDistancia();
  
  TempAndHumidity  data = dhtSensor.getTempAndHumidity();// llamada de datos
  double sensorHS = analogRead(pinsensorHS);
  double HS = map(sensorHS, 0, 4096, 0, 1023);
  double pcHS = map(sensorHS, 0, 4096, 100, 0);
  int humedadTierra = tierraSeca(HS);
  
  imprimirLCD(data.humidity,data.temperature,HS,pcHS);
  
  if(nivelAgua()){
    if(humedadTierra == 3){

        digitalWrite(pinRed,0);
        digitalWrite(pinGreen,0);
        digitalWrite(pinBlue,255);
        
        digitalWrite(ledrojo,LOW);
        digitalWrite(bomba,LOW); // Encender Bomba
    }else{
      if(humedadTierra == 2){
        if(data.temperature>23){
          
          digitalWrite(pinRed,0);
          digitalWrite(pinGreen,0);
          digitalWrite(pinBlue,255);

          digitalWrite(ledrojo,LOW);
          digitalWrite(bomba,LOW); // Encender Bomba
        }else{
         
          digitalWrite(pinRed,0);
          digitalWrite(pinGreen,255);
          digitalWrite(pinBlue,0);
          
          digitalWrite(ledrojo,LOW);
          digitalWrite(bomba,HIGH); // Apagar Bomba
        } 
      }else{
          digitalWrite(pinRed,0);
          digitalWrite(pinGreen,255);
          digitalWrite(pinBlue,0);

          digitalWrite(ledrojo,LOW);
          digitalWrite(bomba,HIGH); // Apagar Bomba
      }
    }
  }else{
    digitalWrite(ledrojo,HIGH);
    digitalWrite(pinRed,0);
    digitalWrite(pinGreen,0);
    digitalWrite(pinBlue,0);
    digitalWrite(bomba,HIGH); // Apagar Bomba
  }
  
  if(data.humidity>=50 && data.humidity<=75){
    digitalWrite(ledMulti,LOW);
  }else{
    digitalWrite(ledMulti,HIGH); 
  }

}
