//
//Requires PubSubClient found here: https://github.com/knolleary/pubsubclient
//Requires DHT from Adafruit here: https://github.com/adafruit/DHT-sensor-library

#include <PubSubClient.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>

#define oneWire18S20_ID 0x10
#define oneWire18B20_ID 0x28
#define DHTTYPE DHT22
#define WINDPIN 3 //Interrupt
#define DHTPIN 4
#define PIRPIN 3
#define SOILTEMPPIN 8
#define SOILHUMPIN 9
#define SOILHUMPANALOGIN A0

// Initialize DHT sensor 
// NOTE: For working with a faster than ATmega328p 16 MHz Arduino chip, like an ESP8266,
// you need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// This is for the ESP8266 processor on ESP-01 
DHT dht(DHTPIN, DHTTYPE, 6); // 11 works fine for ESP8266
OneWire oneWire(SOILTEMPPIN);

float soilTemp;

const int soilTempPin = SOILTEMPPIN;
const int soilHumPin = SOILHUMPIN;

float humidity, temp_c, temp_f;          // Values read from sensor

unsigned long previousMillis = 0;        // will store last temp was read
const long interval = 2000;              // interval at which to read sensor

// einige konstanten 
const int windPin = WINDPIN;          // anschluss des reedkontaktes

const float windFactor = 2.4;   // umrechnungsfaktor von umdrehungen in geschwindigkeit
const int measureTime = 10;      // messzeitraum in sekunden

// variablen
volatile unsigned int windCounter = 0;  //interner zaehler fŸr umdrehungen 
float windSpeed = 0.0;

volatile unsigned int soilHum = 0;
volatile unsigned int soilHumDry = 0;


//initialisieren der variablen fŸr messwerte und die zeitmessung
unsigned long windTime = 0;

const int pirPin = PIRPIN;
volatile unsigned int pirValue = 0;

//EDIT THESE LINES TO MATCH YOUR SETUP
//#define MQTT_SERVER "###.###.###.###"
//const char* ssid = "############";
//const char* password = "##########";


//topic to publish to for the temperature
char* tempTopic = "your temperature topic";
char* currentTemp;

char* humTopic = "your humidity topic";
char currentHum;

//MQTT callback
void callback(char* topic, byte* payload, unsigned int length) {
  
}

//WiFiClient wifiClient;
//PubSubClient client(MQTT_SERVER, 1883, callback, wifiClient);

//networking functions

/*void reconnect() {

  //attempt to connect to the wifi if connection is lost
  if(WiFi.status() != WL_CONNECTED){

    //loop while we wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
    }

  }

  //make sure we are connected to WIFI before attemping to reconnect to MQTT
  if(WiFi.status() == WL_CONNECTED){
  // Loop until we're reconnected to the MQTT server
    while (!client.connected()) {

      // Generate client name based on MAC address and last 8 bits of microsecond counter
      String clientName;
      clientName += "esp8266-";
      uint8_t mac[6];
      WiFi.macAddress(mac);
      clientName += macToStr(mac);

      //if connected, subscribe to the topic(s) we want to be notified about
      if (client.connect((char*) clientName.c_str())) {
        //subscribe to topics here
      }
    }
  }
}

//generate unique name from MAC addr
String macToStr(const uint8_t* mac){

  String result;

  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);

    if (i < 5){
      result += ':';
    }
  }

  return result;
}
*/

void gettemperature() {
  // Wait at least 2 seconds seconds between measurements.
  // if the difference between the current time and last time you read
  // the sensor is bigger than the interval you set, read the sensor
  // Works better than delay for things happening elsewhere also
  unsigned long currentMillis = millis();
  

 
    if(currentMillis - previousMillis >= interval) {
      // save the last time you read the sensor 
      previousMillis = currentMillis;   
 
      // Reading temperature for humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
      humidity = dht.readHumidity();          // Read humidity (percent)
      temp_f = dht.readTemperature(true);     // Read temperature as Fahrenheit
      temp_c = (((temp_f-32.0)*5/9)-1);       // Convert temperature to Celsius + Correction
      // Check if any reads failed and exit early (to try again).
      if (isnan(humidity) || isnan(temp_c)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
      }      
    }
  
}

void getWind(){
  //starten des messzeitraums
  Serial.print("starting wind-counter for ");
  Serial.print(measureTime);
  Serial.println(" seconds");
  //zaehler auf 0 stellen
  windCounter = 0;
  windTime = millis();
  //zaehl-interrupt aktivieren
  attachInterrupt(windPin,countWind,RISING);
  //abwarten des messzeitraums
  delay(100 * measureTime);
  //zaehl-interrupt deaktivieren
  detachInterrupt(windPin);
  //zeit bestimmen
  windTime = (millis() - windTime) / 1000;
  
  //debug ausgaben
  Serial.print("Time elapsed : ");
  Serial.println(windTime);
  Serial.print("Wind counts = ");
  Serial.println(windCounter);
  //berechnen der geschwindigkeit
  Serial.print(windCounter);
  if (windCounter <= 1) {
    windSpeed = 0;
    Serial.print("Wind Speed = ");
    Serial.print(windSpeed);
    Serial.println(" km/h");
  }
  else if (windCounter > 200)
  {
    
  }
  else {
    windSpeed = (float)windCounter/4 / (float)measureTime * windFactor;
    Serial.print("Wind Speed = ");
    Serial.print(windSpeed);
    Serial.println(" km/h");
  }
}

//interrupt service routine fŸr das zaehlen der umdrehungen
void countWind() {
   windCounter ++; 
}

void initWind(){
  //startzeit bestimmen
  windTime = millis();
}

void setup() {

  //null terminate the temp string to be published
  

  //start the serial line for debugging
  Serial.begin(115200);
  dht.begin(); 
  delay(100);
  
  Serial.println("");
  Serial.println("DHT Temp/Hum MQTT Client");

  /*
  //start wifi subsystem
  WiFi.begin(ssid, password);
  WiFi.enableAP(false);
  

  //attempt to connect to the WIFI network and then connect to the MQTT server
  reconnect();
  */

  initPir();
  
  //wait a bit before starting the main loop
  delay(2000);
}

void  initPir(){
  pinMode(pirPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pirPin),getPir,RISING);
}

void getPir(){
  pirValue = digitalRead(pirPin);
  Serial.println(pirValue);
}

void getSoilHum(){
  soilHum = analogRead(SOILHUMPANALOGIN);
  soilHumDry = digitalRead(soilHumPin);
  Serial.println(soilHum);
  Serial.println(soilHumDry);
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
  return true;
}

void loop(){
  pirValue = 0;

  // Send the command to update temperatures
  gettemperature();
  getWind();

  if (getSoilTemp()) {
    Serial.println(soilTemp);
  }
  else { Serial.println("Konnte Bodentemperatur nicht auslesen"); }

  getSoilHum();
 
  // Define 
  String str1 = String(temp_c); 
  // Length (with one extra character for the null terminator)
  int str1_len = str1.length() + 1; 
  // Prepare the character array (the buffer) 
  char char_array_temp[str1_len];
  // Copy it over 
  str1.toCharArray(char_array_temp, str1_len);
  
  String str2;
  if(humidity <= 100){
     str2 = String(int(humidity)); 
  }else{
    str2 = "nan"; 
  }
  int str2_len = str2.length() + 1; 
  char char_array_hum[str2_len];
  str2.toCharArray(char_array_hum, str2_len);

  /*
  //publish the new temperature
  client.publish(tempTopic, char_array_temp);
  client.publish(humTopic, char_array_hum);*/
  Serial.println(String((float)temp_c)+" : "+String((float)temp_f)+" : "+String((int)humidity));
  Serial.println(str2);

  delay(2000);

  /*
  //reconnect if connection is lost
  if (!client.connected() && WiFi.status() == 3) {reconnect();}
  //maintain MQTT connection
  client.loop();*/
  //MUST delay to allow ESP8266 WIFI functions to run 
}
