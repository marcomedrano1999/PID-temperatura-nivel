#include <PID_v1.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#define pinData 2                                                   // Pin donde se conectará el pin de datos.
#define SSR 6
#define led 13                                                
#define tiempoCiclo 1000

//Variables de temperatura----------------------------------------------------------
double Setpoint, Input, Output;                                     // Define Variables 
double Kp=10, Ki=3, Kd=400;                                         // Especifica parametros iniciales
float temperatura=0;

unsigned long respuestaUltimaTemperatura = 0;
unsigned long lastPIDCalculation = 0;
float prevTemperature = -9999.0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
OneWire ourWire(pinData);                                           // Pin como bus para la comunicación OneWire
DallasTemperature sensors(&ourWire);                                //Se instancia la librería DallasTemperature


//Variables de flujo----------------------------------------------------------------
volatile int NumPulsos; //variable para la cantidad de pulsos recibidos
int PinSensor = 2;    //Sensor conectado en el pin 2
float factor_conversion=7.11; //para convertir de frecuencia a caudal
float volumen=0;
long dt=0; //variación de tiempo por cada bucle
long t0=0; //millis() del bucle anterior


//---Función que se ejecuta en interrupción---------------
void ContarPulsos ()  
{ 
  NumPulsos++;  //incrementamos la variable de pulsos
} 

//---Función para obtener frecuencia de los pulsos--------
int ObtenerFrecuecia() 
{
  int frecuencia;
  NumPulsos = 0;   //Ponemos a 0 el número de pulsos
  interrupts();    //Habilitamos las interrupciones
  delay(1000);   //muestra de 1 segundo
  noInterrupts(); //Deshabilitamos  las interrupciones
  frecuencia=NumPulsos; //Hz(pulsos por segundo)
  return frecuencia;
}


void setup()
{
  Setpoint = 40.0;                                                 // initialize the variables we're linked to
  myPID.SetOutputLimits(0, tiempoCiclo);                           
  myPID.SetSampleTime(tiempoCiclo);
  myPID.SetMode(AUTOMATIC);                                       
  Serial.begin(115200);                                             // Aranca comunicacion serie

  //Serial.begin(9600);                                           //Modificar afecta al controlador?
  
  pinMode(SSR, OUTPUT);
  digitalWrite(SSR, LOW); 
   pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  sensors.begin();                                                // arranca librerias


  //setup de medición de flujo-----------------------------------------------------
  Serial.begin(9600); 
  pinMode(PinSensor, INPUT); 
  attachInterrupt(0,ContarPulsos,RISING);//(Interrupción 0(Pin2),función,Flanco de subida)
  Serial.println ("Envie 'r' para restablecer el volumen a 0 Litros"); 
  t0=millis();
}

void loop()
{
  //Control de nivel----------------------------------------------------------------------
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float nivel = sensorValue * (5.0 / 1023.0); /*aqui se haria la conversion dependiendo del voltaje
  que se arroje del potenciometro que se mueve con el flotador, pero habria que hacer pruebas*/
  // print out the value you read:
  Serial.println(nivel);


  //Medición de flujo---------------------------------------------------------------------
  if (Serial.available()) {
    if(Serial.read()=='r')volumen=0;//restablecemos el volumen si recibimos 'r'
  }
  float frecuencia=ObtenerFrecuecia(); //obtenemos la frecuencia de los pulsos en Hz
  float caudal_L_m=frecuencia/factor_conversion; //calculamos el caudal en L/m
  dt=millis()-t0; //calculamos la variación de tiempo
  t0=millis();
  volumen=volumen+(caudal_L_m/60)*(dt/1000); // volumen(L)=caudal(L/s)*tiempo(s)

   //-----Enviamos por el puerto serie---------------
  Serial.print ("Caudal: "); 
  Serial.print (caudal_L_m,3); 
  Serial.print ("L/min\tVolumen: "); 
  Serial.print (volumen,3); 
  Serial.println (" L");

  

 //Control de temperatura-----------------------------------------------------------------
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
