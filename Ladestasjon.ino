#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const char* ssid = "Penthouse"; //SSID må tastes inn
const char* password = "Quaintriver135"; //Password må tastes inn

// mqtt_server tar imot RPi sin ip-adresse. Eksempel: 172.20.10.10
const char* mqtt_server = "192.168.1.172"; // Raspberry Pi IP må tastes inn

WiFiClient espClient;
PubSubClient client(espClient);

const int trigPin = 23; // Avstandssensor
const int echoPin = 19; // Avstandssensor

// Funksjon som returnerer verdiene fra avstandssensoren i cm
float readDistance() {  
  
  // Tilstandene endrer seg hele tiden for å måle bølgelengden.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Leser echoPin, returnerer tid i mikrosekunder
  long duration = pulseIn(echoPin, HIGH);

  float distanceCm = duration * 0.034/2;
  return distanceCm; // Returnerer avstand i cm
}

// Funksjon som sjekker om bil er foran sensor
void isCarPresent() { 
  float distance = readDistance(); // Henter verdi fra avstandssensor
  
  if (distance < 20) { // Hvis bil er tilstede-> sender 1 til NodeRED
    client.publish("esp32/ladestasjon", "1"); // Publiserer "1" til NodeRED under topic "esp32/ladestasjon"
  }
}

// NodeMCU kobler til Wi-Fi
void setup_wifi() {
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

// Funksjon der ESP-32 subscriber til Node-Red
void callback(String topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageInfo;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageInfo += (char)message[i];
  }
  Serial.println();

  // Kode for subscribe. Sjekker om topic endrer seg
  if (topic == "esp32/displaybattery") { // Henter batteriprosent til bilen, via NodeRed
    display.clearDisplay();
    display.setTextSize(3);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println(messageInfo);
    display.display();
    }
  else { // Standardmelding ved ingen lading
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("Velkommen til ladestasjonen!");
    display.display();
    }
  }

// Funksjon som reconnecter helt til vi er tilkoblet MQTT.
void reconnect() {
  // Loop helt til vi er tilkoblet MQTT
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");


    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      client.subscribe("esp32/displaybattery");
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

//Setter opp GPIO. Callback kjøres i void setup, ikke i void loop.
void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT); // Avstandssensor
  pinMode(echoPin, INPUT); // Avstandssensor
  setup_wifi();
  client.setServer(mqtt_server, 1883); //1883 er standard port for MQTT kommunikasjon
  client.setCallback(callback);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // OLED-display
  Serial.println("SSD1306 allocation failed");
  for(;;); // Don't proceed, loop forever
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Setter standard melding
  display.println("Velkommen til ladestasjonen!");
  display.display();

}

// Reconnecter til wifi hvis nødvendig
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  if (!client.loop()) {
    client.connect("ESP8266Client");
  }
  isCarPresent(); //Sjekker om bil er foran sensor, gir ut tilstand 1 dersom bil er tilstede
  delay(10);
}
