// Libraries
#include <SFE_BMP180.h>
#include <SPI.h>
#include <Ethernet.h>
#include <aREST.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <OneWire.h>
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


//------------------------Pins--------------------------------

#define oneWire18S20_ID 0x10
#define oneWire18B20_ID 0x28
#define DHTTYPE DHT22
#define WINDPIN 3 //Interrupt
#define PIRPIN 2 //Interrupt
#define DHTPIN 6
#define SOILTEMPPIN 5
#define SOILMOISPIN 22
#define BRIGHTNESSPIN 9
#define BRIGHTNESSANALOGPIN A10
#define SOILMOISANALOGPIN A0
#define RAIN1PIN 7
#define RAIN1ANALOGPIN A8
#define RAIN2PIN 8
#define RAIN2ANALOGPIN A9
#define SWITCH1PIN 30
#define SWITCH2PIN 30

//-----------------------const---------------------------------

const float windFactor = 2.4;   // umrechnungsfaktor von umdrehungen in geschwindigkeit
const int measureTime = 10;     // messzeitraum in sekunden

//--------------------- variablen------------------------------
volatile unsigned int windCounter = 0;  //interner zaehler für umdrehungen 
float windSpeed = 0.0;

float airTemp = 0.0;
float airHum = 0.0;
float airPres = 0.0;

float soilTemp = 0.0;
float soilMois = 0.0;

int motion = 0;
int bright = 0;
int brightness = 0;
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

//-------------------------DHT-Setup-------------------------------
// Initialize DHT sensor 
// NOTE: For working with a faster than ATmega328p 16 MHz Arduino chip, like an ESP8266,
// you need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// This is for the ESP8266 processor on ESP-01 
DHT dht(DHTPIN, DHTTYPE, 6); // 11 works fine for ESP8266

//--------------------------BMP180---------------------------------

SFE_BMP180 pressure;

//--------------------------OneWire---------------------------------

OneWire oneWire(SOILTEMPPIN);

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

  rest.variable("motion_detector", &motion);

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
  pinMode(WINDPIN, INPUT_PULLUP);

  // BMP180 init
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    Serial.println("BMP180 init fail\n\n");
  }

  //motion detector init
  pinMode(PIRPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIRPIN),getPir,RISING);

  //pinmode init
  pinMode(SWITCH1PIN, OUTPUT);
  digitalWrite(SWITCH1PIN, HIGH);
  pinMode(SWITCH2PIN, OUTPUT);
  digitalWrite(SWITCH2PIN, HIGH);
}

void startMeasureWind(){
  //starten des messzeitraums
  Serial.print("starting wind-counter for ");
  Serial.print(measureTime);
  Serial.println(" seconds");
  //zaehler auf 0 stellen
  windCounter = 0;
  time = millis();
  //zaehl-interrupt aktivieren
  attachInterrupt(digitalPinToInterrupt(WINDPIN),countWind,FALLING);
}

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

float getAirTemp(){
  float temp_f = dht.readTemperature(true);     // Read temperature as Fahrenheit
  float temp_c = (((temp_f-32.0)*5/9)-1);       // Convert temperature to Celsius + Correction

  if (isnan(temp_c)){
    temp_f = dht.readTemperature(true);     // if read fails try one more time
    temp_c = (((temp_f-32.0)*5/9)-1);
  }
  if (isnan(temp_c)){
    return -100.0;
  }
  Serial.print("Air Temperature: ");
  Serial.println(temp_c);
  return temp_c;
}

int getAirHum(){
  int humidity = dht.readHumidity();       // Read humidity (percent)
  if(isnan(humidity)){
    humidity = dht.readHumidity();
  }
  if(isnan(humidity)){
    return -1;
  }
  Serial.print("Air Humidity: ");
  Serial.println(humidity);
  return humidity;
}

boolean getSoilTemp(){
  oneWire.reset_search();
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  //find a device
  if (!oneWire.search(addr)) {
    oneWire.reset_search();
    return false;
  }
  
  if (OneWire::crc8( addr, 7) != addr[7]) {
    return false;
  }
   
  if (addr[0] != oneWire18S20_ID && addr[0] != oneWire18B20_ID) {
    return false;
  }
   
  oneWire.reset();
  oneWire.select(addr);
  // Start conversion
  oneWire.write(0x44, 1);
  // Wait some time...
  delay(850);
  present = oneWire.reset();
  oneWire.select(addr);
  // Issue Read scratchpad command
  oneWire.write(0xBE);
  // Receive 9 bytes
  for ( i = 0; i < 9; i++) {
    data[i] = oneWire.read();
  }
  // Calculate temperature value
  soilTemp = ( (data[1] << 8) + data[0] )*0.0625;
  Serial.print("Soil Temperature: ");
  Serial.println(soilTemp);
  return true;
}

double getSoilMois(){
  double soilMois = analogRead(SOILMOISANALOGPIN);
  boolean soilMoisDry = digitalRead(SOILMOISPIN);
  Serial.print("Soil Moisture: ");
  Serial.println(soilMois);
  Serial.print("Moist: ");
  Serial.println(soilMoisDry);
  return soilMois;
}

int getBrightness(){
  brightness = analogRead(BRIGHTNESSANALOGPIN);
  bright = digitalRead(BRIGHTNESSPIN);
  Serial.print("Brightness: ");
  Serial.println(brightness);
  Serial.print("Bright: ");
  Serial.println(bright);
  return soilMois;
}

int getPrecip(){
  int rain1 = ! digitalRead(RAIN1PIN);
  int rain2 = ! digitalRead(RAIN2PIN);

  int rain1a = analogRead(RAIN1ANALOGPIN);
  int rain2a = analogRead(RAIN2ANALOGPIN);
  Serial.print("Raining: ");
  Serial.println(rain1 && rain2);
  return (rain1 && rain2);
}

void getPir(){
  motion = digitalRead(PIRPIN);
  Serial.println("Motion detected!");
}

double getPressure(){
  char status;
  double T,P,p0,a;

  // Loop here getting pressure readings every 10 seconds.

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("temperature: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(" deg F");
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("absolute pressure: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
  return P;
}

int switch1(String param){
  Serial.println("switch1.param : "+param);

  // Get state from param
  if (param == "n"){
      digitalWrite(SWITCH1PIN, LOW);
      switch1_state = true;
      Serial.println("switch1 : on");
  }else if (param == "ff"){
      digitalWrite(SWITCH1PIN, HIGH);
      switch1_state = false;
      Serial.println("switch1 : off");
  }
}

int switch2(String param){
  Serial.println("switch2.param : "+param);

  // Get state from param
  if (param == "n"){
      digitalWrite(SWITCH2PIN, LOW);
      switch2_state = true;
      Serial.println("switch2 : on");
  }else if (param == "ff"){
      digitalWrite(SWITCH2PIN, HIGH);
      switch2_state = false;
      Serial.println("switch2 : off");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  // listen for incoming clients
  EthernetClient client = server.available();
  rest.handle(client);

  //read only every 10 sec
  if(timer == 20){
    // stop interupt
    stopMeasureWind();

    airTemp = getAirTemp();
    airHum = getAirHum();
    soilMois = getSoilMois();
    soilTemp = getSoilTemp();
    brightness = getBrightness();
    precip = getPrecip();
    airPres = getPressure();

    Serial.print("Motion: ");
    Serial.println(motion);

    //reset motion
    motion = 0;
    
    //reset timer
    timer = 0;

    Serial.println("");
    Serial.println("#######################");
    Serial.println("");
    
    startMeasureWind();
  }
  
  delay(500); //0.5 sec wait
  timer++;
}
