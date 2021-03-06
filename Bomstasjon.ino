//Kode for bomstasjon

#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const char* ssid = "SSID"; //SSID må tastes inn
const char* password = "PASSORD"; //Password må tastes inn

// mqtt_server tar imot RPi sin ip-adresse. Eksempel: 172.20.10.10
const char* mqtt_server = "192.168.1.172"; //Raspberry pi IP må tastes inn

WiFiClient espClient;
PubSubClient client(espClient);

const int trigPin = 23; // Til avstandssensor
const int echoPin = 19; // Til avstandssensor
const int servoPin = 26; // Til bomstasjon
const int redLed = 5; // Rødt trafikklys
const int greenLed = 18; // Grønt trafikklys

Servo servo;

float readDistance() { // Funksjon som leser av distanse ved hjelp av avstandssensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distanceCm = duration * SOUND_SPEED/2;
  return(distanceCm);
  
  delay(100);
}

void setup_wifi() { // Funksjon som kobler ESP32 til WiFi ved hjelp av WiFi bibliotek
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - NodeMCU IP address: ");
  Serial.println(WiFi.localIP());
}


void reconnect() { //Funksjon som prøver å koble ESP32 til MQTT
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


void setup() { //Definerer alle inputs og outputs
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  servo.attach(servoPin); //Setter opp servo
  setup_wifi(); //Setter opp WiFi
  client.setServer(mqtt_server, 1883); //Setter opp MQTT Server. 1883 er standard port for MQTT kommunikasjon
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Setter opp OLED-display
  Serial.println("SSD1306 allocation failed");
  for(;;);
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("Bomavgift: 100kr");
  display.display(); 
  digitalWrite(redLed, HIGH); //Rødt lys vises som standard
  digitalWrite(greenLed, LOW);
}

unsigned long lapTimeMillis = 0; // Timer for hvor lenge bommen skal være oppe
float lapTime; // Variabel som skal vise rundetiden
bool servoCount = 0; // Variabel som teller antall ganger servoen har vært oppe. Nødvendig for å regne ut rundetiden

void loop() {
  if (!client.connected()) { //If-setning som prøver å reconnecte til MQTT hvis nødvendig
    reconnect();
  }
  if (!client.loop()) { // Kobler til ESP-klienten
    client.connect("ESP8266Client");
  }
  
  float distanceMeasure = readDistance(); // Variabel som lagrer distanseverdi
  static char distanceString[7]; // Opretter char array
  dtostrf(distanceMeasure, 6, 2, distanceString); // Konverterer float til char array
  client.publish("esp32/distance", distanceString); //Publiserer distanse til topic "esp32/distance". Distance må være char array

  if (distanceMeasure < 10) //Hvis kjøretøyet er nærmere enn 10 cm blir det grønt lys, og bommen åpner seg.
    servo.write(90);
    digitalWrite(greenLed, HIGH); //Grønt led-lys når bommen er oppe
    digitalWrite(redLed, LOW);
    if (servoCount == 0) {
      lapTimeMillis = millis(); //Starter timer hvis den passerer en gang
    }
    if (servoCount == 1) {
      lapTime = ((unsigned long)millis() - lapTimeMillis)/1000.0; //Slutter timer hvis den passerer to ganger
    }
    delay(3000); //Bommen er oppe i 3 sekunder
    servo.write(0); //Lukker bommen
    digitalWrite(redLed, HIGH); //Rødt led-lys når bommen er nede
    digitalWrite(greenLed, LOW);
    servoCount = !servoCount; //Inverterer servoCount
    static char racingString[7]; //Lager en char array for rundetiden
    dtostrf(lapTime, 6, 2, racingString); //Konverterer rundetid fra float til char array
    client.publish("esp32/racingtime", racingString); //Publiserer rundetid med topic "esp32/racingtime"
    delay(100);
} 
 
