/*
 * ESP32-kode for busstopp 1 og 2
 */


#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Penthouse"; // SSID må tastes inn
const char* password = "Quaintriver135"; // Password må tastes inn

// mqtt_server tar imot RPi sin ip-adresse. Eksempel: 172.20.10.10
const char* mqtt_server = "192.168.1.172"; // Raspberry Pi IP må tastes inn

WiFiClient espClient;
PubSubClient client(espClient);

const int trigPin1 = 23; // Busstopp 1
const int echoPin1 = 19; // Busstopp 1
const int trigPin2 = 21; // Busstopp 2
const int echoPin2 = 22; // Busstopp 2

/* Funksjon som henter verdiene fra avstandssensor 1 i cm
 * Sjekker om bil er tilstede, og skriver til NodeRED
 */
float busstopp1() { // 
  // Tilstandene endrer seg hele tiden for å måle bølgelengden.
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  // Leser echoPin, returnerer tid i mikrosekunder
  long duration = pulseIn(echoPin1, HIGH);

  float distanceCm1 = duration * 0.034/2;

  if (distanceCm1 < 20) { // Sjekker om avstand er under 20 cm
      client.publish("esp32/busstop1", "1");
  }
}

/* Funksjon som henter verdiene fra avstandssensor 2 i cm
 * Sjekker om bil er tilstede, og skriver til NodeRED
 */
void busstopp2() {
  // Tilstandene endrer seg hele tiden for å måle bølgelengden.
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  // Leser echoPin, returnerer tid i mikrosekunder
  long duration = pulseIn(echoPin2, HIGH);

  float distanceCm2 = duration * 0.034/2;

  if (distanceCm2 < 20) { // Sjekker om avstand er under 20 cm
      client.publish("esp32/busstop2", "1");
  }
}

//NodeMCU kobler til Wi-Fi
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

// Funksjon som reconnecter helt til vi er tilkoblet MQTT.
void reconnect() {
  // Loop helt til vi er tilkoblet MQTT
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

//Setter opp GPIO. Callback kjøres i void setup, ikke i void loop.
void setup() {
  Serial.begin(115200);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  setup_wifi();
  client.setServer(mqtt_server, 1883); //1883 er standard port for MQTT kommunikasjon
}

// Reconnecter til wifi hvis nødvendig
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  if (!client.loop()) {
    client.connect("ESP8266Client");
  }
  busstopp1(); // Kjører funksjonen som sjekker om bil er tilstede på busstopp1
  busstopp2(); // Kjører funksjonen som sjekker om bil er tilstede på busstopp2
  delay(50);
}
