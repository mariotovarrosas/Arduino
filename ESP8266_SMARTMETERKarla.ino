/*
  WriteMultipleFields
  
  Description: Writes values to fields 1,2,3,4 and status in a single ThingSpeak update every 20 seconds.
  
  Hardware: Arduino compatible hardware controlling ESP8266 through AT commands. THIS IS CODE FOR THE ARDUINO, NOT THE ESP8266!
  
  !!! IMPORTANT - Modify the secrets.h file for this project with your network connection and ThingSpeak channel details. !!!
  
  Note:
  - Requires Arduino main board connected to ESP8266 (ESP-01) serial pins
  - The ESP8266 device must be running firmware capable of AT commands over TX/RX pins. 
      Details on reflashing and binaries can be found here: https://www.espressif.com/en/support/download/at  
  - Requires WiFiEsp library which is available through the Library Manager. The logging level for the WiFiEsp library is set to INFO.
      To disable logging completely, set _ESPLOGLEVEL_ to 0 in \Arduino\libraries\WiFiEsp\src\utility\debug.h
  - Use TX1/RX1 if present on Arduino header (Mega, Due, etc). If not, then connect TX to pin 7, RX to pin 6.
  - Some boards (Uno, Nano, Leonardo, etc) do not have enough memory to handle writing large strings to ThingSpeak.
      For these boards, keep individual field data lengths 32 characters or less. 
  - This example is written for a network using WPA encryption. For WEP or WPA, change the WiFi.begin() call accordingly.
  
  Wiring diagrams are available at:
  SoftSerial (Uno, Nano, Mini, etc): https://github.com/mathworks/thingspeak-arduino/blob/master/ESP-01_AT_Commands_SoftSerial_Hookup.pdf
  Hardware Serial1 (Mega, Leonardo, Due) - https://github.com/mathworks/thingspeak-arduino/blob/master/ESP-01_AT_Commands_Hardware_Serial_Hookup.pdf
  
    ESP8266 | Arduino without Serial1 | Arduino with Serial1  
    --------------------------------------------------------
       RX   |         pin 7           |        TX1
       TX   |         pin 6           |        RX1
       GND  |          GND            |        GND
       VCC  |          5V             |        5V
      CH_PD |          5V             |        5V
      
  ThingSpeak ( https://www.thingspeak.com ) is an analytic IoT platform service that allows you to aggregate, visualize, and 
  analyze live data streams in the cloud. Visit https://www.thingspeak.com to sign up for a free account and create a channel.  
  
  Documentation for the ThingSpeak Communication Library for Arduino is in the README.md folder where the library was installed.
  See https://www.mathworks.com/help/thingspeak/index.html for the full ThingSpeak documentation.
  
  For licensing information, see the accompanying license file.
  
  Copyright 2019, The MathWorks, Inc.
*/

#include "ThingSpeak.h"
#include "WiFiEsp.h"
#include "secrets.h"
#include <Filters.h> //Easy library to do the calculations
#include <Wire.h>

char ssid[] = SECRET_SSID;   // your network SSID (name) 
char pass[] = SECRET_PASS;   // your network password
int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiEspClient  client;

// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(3, 2); // RX, TX
#define ESP_BAUDRATE  19200
#else
#define ESP_BAUDRATE  115200
#endif

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

float testFrequency = 60;                     // test signal frequency (Hz)
float windowLength = 40.0/testFrequency;     // how long to average the signal, for statistist

int Sensor = 0; //Sensor analog input, here it's A0

float intercept = -0.04; // to be adjusted based on calibration testing
float slope = 0.0405; // to be adjusted based on calibration testing
float current_Volts; // Voltage
float voltajeSensor;
float corriente=0;
float Sumatoria=0;
long tiempo=millis();
int N=0;
unsigned long printPeriod = 1000; //Refresh rate
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
const unsigned long intervalo=45000;
// This function attempts to set the ESP8266 baudrate. Boards with additional hardware serial ports
// can use 115200, otherwise software serial is limited to 19200.
void setEspBaudRate(unsigned long baudrate){
  long rates[6] = {115200,74880,57600,38400,19200,9600};

  Serial.print("Setting ESP8266 baudrate to ");
  Serial.print(baudrate);
  Serial.println("...");

  for(int i = 0; i < 6; i++){
    Serial1.begin(rates[i]);
    delay(100);
    Serial1.print("AT+UART_DEF=");
    Serial1.print(baudrate);
    Serial1.print(",8,1,0,0\r\n");
    delay(100);  
  }
    
  Serial1.begin(baudrate);
}

float get_corriente()
{
  float voltajeSensor;
  float corriente=0;
  float Sumatoria=0;
  long tiempo=millis();
  int N=0;
  while(millis()-tiempo<500)//Duración 0.5 segundos(Aprox. 30 ciclos de 60Hz)
  { 
    voltajeSensor = analogRead(A0)* (1.1 / 1023.0);//voltaje del sensor de corriente alterna
    corriente=voltajeSensor*185.0; //corriente=VoltajeSensor*(100A/1V)
    Sumatoria=Sumatoria+sq(corriente);//Sumatoria de Cuadrados
    N=N+1;
    delay(1);  
  }
  Sumatoria=Sumatoria*2;//Para compensar los cuadrados de los semiciclos negativos.
  corriente=sqrt((Sumatoria)/N); //Ecuación del RMS
  corriente=corriente;
  return(corriente);
}

float get_corriente2()
{
  float voltajeSensor2;
  float corriente2=0;
  float Sumatoria2=0;
  long tiempo2=millis();
  int N2=0;
  while(millis()-tiempo2<500)//Duración 0.5 segundos(Aprox. 30 ciclos de 60Hz)
  { 
    voltajeSensor2 = analogRead(A2)* (1.1 / 1023.0);//voltaje del sensor de corriente alterna
    corriente2=voltajeSensor2*187.0; //corriente=VoltajeSensor*(100A/1V)
    Sumatoria2=Sumatoria2+sq(corriente2);//Sumatoria de Cuadrados
    N2=N2+1;
    delay(1);  
  }
  Sumatoria2=Sumatoria2*2;//Para compensar los cuadrados de los semiciclos negativos.
  corriente2=sqrt((Sumatoria2)/N2); //Ecuación del RMS
  corriente2=corriente2;
  return(corriente2);
}

float get_corriente3()
{
  float voltajeSensor3;
  float corriente3=0;
  float Sumatoria3=0;
  long tiempo3=millis();
  int N3=0;
  while(millis()-tiempo3<500)//Duración 0.5 segundos(Aprox. 30 ciclos de 60Hz)
  { 
    voltajeSensor3 = analogRead(A3)* (1.1 / 1023.0);//voltaje del sensor de corriente alterna
    corriente3=voltajeSensor3*157.0; //corriente=VoltajeSensor*(100A/1V)
    Sumatoria3=Sumatoria3+sq(corriente3);//Sumatoria de Cuadrados
    N3=N3+1;
    delay(1);  
  }
  Sumatoria3=Sumatoria3*2;//Para compensar los cuadrados de los semiciclos negativos.
  corriente3=sqrt((Sumatoria3)/N3); //Ecuación del RMS
  corriente3=corriente3;
  return(corriente3);
}
float get_corriente4()
{
  float voltajeSensor4;
  float corriente4=0;
  float Sumatoria4=0;
  long tiempo4=millis();
  int N4=0;
  while(millis()-tiempo4<500)//Duración 0.5 segundos(Aprox. 30 ciclos de 60Hz)
  { 
    voltajeSensor4 = analogRead(A4)* (1.1 / 1023.0);//voltaje del sensor de corriente alterna
    corriente4=voltajeSensor4*127.0; //corriente=VoltajeSensor*(100A/1V)
    Sumatoria4=Sumatoria4+sq(corriente4);//Sumatoria de Cuadrados
    N4=N4+1;
    delay(1);  
  }
  Sumatoria4=Sumatoria4*2;//Para compensar los cuadrados de los semiciclos negativos.
  corriente4=sqrt((Sumatoria4)/N4); //Ecuación del RMS
  corriente4=corriente4;
  return(corriente4);
}

void(* resetFunc) (void) = 0;//declare reset function at address 0


void setup() {
  //Initialize serial and wait for port to open
  Serial.begin(115200);  // Initialize serial
  
  // initialize serial for ESP module  
  setEspBaudRate(ESP_BAUDRATE);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo native USB port only
  }

  Serial.print("Searching for ESP8266..."); 
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }
  Serial.println("found it!");
  ThingSpeak.begin(client);  // Initialize ThingSpeak
  pinMode(7, OUTPUT);      // set the LED pin mode

}

void loop() {

digitalWrite(7, LOW);
unsigned long ahora=millis();
if (ahora - previousMillis2 >= intervalo){  

if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(1000);     
    }
    Serial.println("\nConnected.");
  }
  
RunningStatistics inputStats;                //Easy life lines, actual calculation of the RMS requires a load of coding
inputStats.setWindowSecs( windowLength );
float Irms=get_corriente();
float Irms2=get_corriente2();
float Irms3=get_corriente3();
int var=0; 
while( var<3 ) {    
    Sensor = analogRead(A5);  // read the analog in value:
    inputStats.input(Sensor);  // log to Stats function
     
    if((unsigned long)(millis() - previousMillis) >= printPeriod) {
      previousMillis = millis();   // update time every second
      current_Volts = intercept + slope * inputStats.sigma(); //Calibartions for offset and amplitude
      current_Volts= current_Volts*(36.5231);                //Further calibrations for the amplitude
      if (var==0 || var==1){
      var++;
      digitalWrite(7, HIGH);
      }
      else{    
      var++;}
      float Irmstot=Irms+Irms2;
      float Prms=current_Volts*Irms+Irms2;
      float Prms_g=current_Volts*Irms3;
      if (current_Volts<=1){
        current_Volts=0;}
      ThingSpeak.setField(1, current_Volts);
      ThingSpeak.setField(2, Irms3);
      ThingSpeak.setField(3, Prms_g);
      ThingSpeak.setField(4, Irmstot);
      ThingSpeak.setField(5, Prms);
      int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      digitalWrite(7, LOW);
      //delay(26000);
      resetFunc(); //call reset 
      previousMillis2=ahora;}
    }
  }
}
