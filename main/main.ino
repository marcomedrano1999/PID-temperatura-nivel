#include <PID_v1.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#define pinData 2                                                   // Pin donde se conectará el pin de datos.
#define SSR 6
#define led 13                                                
#define tiempoCiclo 1000
double Setpoint, Input, Output;                                     // Define Variables 
double Kp=10, Ki=3, Kd=400;                                         // Especifica parametros iniciales
float temperatura=0;

unsigned long respuestaUltimaTemperatura = 0;
unsigned long lastPIDCalculation = 0;
float prevTemperature = -9999.0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
OneWire ourWire(pinData);                                           // Pin como bus para la comunicación OneWire
DallasTemperature sensors(&ourWire);                                //Se instancia la librería DallasTemperature

void setup()
{
  Setpoint = 40.0;                                                 // initialize the variables we're linked to
  myPID.SetOutputLimits(0, tiempoCiclo);                           
  myPID.SetSampleTime(tiempoCiclo);
  myPID.SetMode(AUTOMATIC);                                       
  Serial.begin(115200);                                             // Aranca comunicacion serie
  pinMode(SSR, OUTPUT);
  digitalWrite(SSR, LOW); 
   pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  sensors.begin();                                                // arranca librerias
}

void loop()
{
  
 if (millis() - respuestaUltimaTemperatura >= tiempoCiclo) {
    temperatura = sensors.getTempCByIndex(0);
    Input = (double)temperatura;
   
    myPID.Compute();
    lastPIDCalculation = millis();
     Serial.print(temperatura);
     Serial.print(" , ");
     Serial.println(Output/50);    
    sensors.requestTemperatures();
    respuestaUltimaTemperatura = millis();
 }
 control();
}

void control() {

    if ((millis() <= (lastPIDCalculation + Output)) || (Output == tiempoCiclo)) {
    // Power on:
    digitalWrite(SSR, HIGH);
    digitalWrite(led, HIGH);
  } else {
    // Power off:
    digitalWrite(SSR, LOW);
    digitalWrite(led, LOW);
  }

//  delay(10);

  
}
