// Libraries
#include <SPI.h>
#include <Ethernet.h>
#include <aREST.h>
#include <avr/wdt.h>

//----------------------network--------------------------------
// Enter a MAC address for your controller below.
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x40 };

// IP address in case DHCP fails
IPAddress ip(192,168,0,2);
// Ethernet server
EthernetServer server(80);

//-----------------------rest---------------------------------
// Create aREST instance
aREST rest = aREST();


//-----------------------const---------------------------------

const int switch1Pin = 13;
const int switch2Pin = 5;
const int windPin = 3;          // anschluss des reedkontaktes

const float windFactor = 2.4;   // umrechnungsfaktor von umdrehungen in geschwindigkeit
const int measureTime = 10;      // messzeitraum in sekunden


//--------------------- variablen------------------------------
volatile unsigned int windCounter = 0;  //interner zaehler fŸr umdrehungen 
float windSpeed = 0.0;

float airTemp = 0.0;
float airHum = 0.0;
float airPres = 0.0;

float soilTemp = 0.0;
float soilMois = 0.0;

int bright = 0;
int precip = 0;

bool switch1_state = false;
bool switch2_state = false;

//initialisieren der variablen für messwerte und die zeitmessung
unsigned long time = 0;

int timer = 1;


//--------------------------interupts------------------------------
//interrupt service routine fŸr das zaehlen der umdrehungen
void countWind() {
   windCounter ++; 
}

//----------------------------main----------------------------------

//initialisierung
void setup() {
  
  // Start Serial
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Starting Weatherstation");
  Serial.println("");
  
  // rest init
  rest.variable("wind_speed",&windSpeed);
  rest.variable("air_temperature",&airTemp);
  rest.variable("air_humidity",&airHum);
  rest.variable("air_pressure", &airPres);
  
  rest.variable("soil_temperature", &soilTemp);
  rest.variable("soil_moisture", &soilMois);  

  rest.variable("brightness", &bright);
  rest.variable("precipation", &precip);

  rest.function("switch1",switch1);
  rest.function("switch2",switch2);
  
  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("000001");
  rest.set_name("arduino_weatherstation");

  //ethernet init
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip);
  }
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  //Anemometer init
  time = millis();
  pinMode(windPin, INPUT_PULLUP);

  //pinmode init
  pinMode(switch1Pin, OUTPUT);
  pinMode(switch2Pin, OUTPUT);
}

//--------------------Start Measure Wind----------------------

void startMeasureWind(){
  //starten des messzeitraums
  Serial.print("starting wind-counter for ");
  Serial.print(measureTime);
  Serial.println(" seconds");
  //zaehler auf 0 stellen
  windCounter = 0;
  time = millis();
  //zaehl-interrupt aktivieren
  attachInterrupt(digitalPinToInterrupt(windPin),countWind,FALLING);
}

//-------------------Stop Measure Wind----------------------

void stopMeasureWind(){
  //zaehl-interrupt deaktivieren
  detachInterrupt(1);
  //zeit bestimmen
  time = (millis() - time) / 1000;
  
  Serial.print("Time elapsed : ");
  Serial.println(time);
  Serial.print("Wind counts = ");
  Serial.println(windCounter);
  //berechnen der geschwindigkeit
  windSpeed = (float)windCounter/4 / (float)measureTime * windFactor;
  Serial.print("Wind Speed = ");
  Serial.print(windSpeed);
  Serial.println(" km/h");
}

//--------------------Rest Switch1-------------------------

int switch1(String param){
  Serial.println("switch1.param : "+param);

  // Get state from param
  if (param == "n"){
      digitalWrite(13, HIGH);
      switch1_state = true;
      Serial.println("switch1 : on");
  }else if (param == "ff"){
      digitalWrite(13, LOW);
      switch1_state = false;
      Serial.println("switch1 : off");
  }
}

//--------------------Rest Switch2-------------------------

int switch2(String param){
  Serial.println("switch2.param : "+param);

  // Get state from param
  if (param == "n"){
      digitalWrite(13, HIGH);
      switch2_state = true;
      Serial.println("switch2 : on");
  }else if (param == "ff"){
      digitalWrite(13, LOW);
      switch2_state = false;
      Serial.println("switch2 : off");
  }
}

//-----------------------loop-------------------------

void loop() {
  // put your main code here, to run repeatedly:

  // listen for incoming clients
  EthernetClient client = server.available();
  rest.handle(client);

  //read only every 10 sec
  if(timer == 20){
    // stop interupt
    stopMeasureWind();


    
    //reset timer
    timer = 0;
    startMeasureWind();
  }
  
  delay(500); //0.5 sec wait
  timer++;
}
