#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Penger Er Billig";          //SSID på nettverket
const char* password = "Tatata ta ta"; //Password på nettverket

// mqtt_server tar imot RPi sin ip-adresse
const char* mqtt_server = "192.168.1.172"; //Rasberry-pi sin ip

String speedReadingString = "0";        // Data som blir lest i fra Zumo programmet via serial kommunikasjon
String batteryReadingString = "100";
String distanceReadingString = "0";
String maxSpeedReadingString = "0";
String averageSpeedString = "0";
String timeOverSeventyProsentSpeed = "0";

WiFiClient espClient;
PubSubClient client(espClient);

int fast_charge;                       // Data hentet i fra nodeRed og sendt til zumo via serial kommunikasjon
int stopZumo;
int chargeZumo;
int wantedCharge;
int nermeLadeStasjon;

unsigned long sendDataDelay = 0;      // millis variabler slik at data ikke blir sendt / motatt flere hundre ganger i sekundet
unsigned long mqttDataDelay = 0;


void setup_wifi() {
    delay(10);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
    }
}


void callback(String topic, byte* message, unsigned int length){
    
    String messageInfo;
    for (int i = 0; i < length; i++) {
        messageInfo += (char)message[i];
    }

    // Henter data i fra nodeRed    
   
    if(topic == "esp32/backchargefast"){
        if(messageInfo == "1") {
            fast_charge = 1;        
        }   
        if(messageInfo == "0") {
            fast_charge = 0;
        }
    }
    if(topic == "esp32/stopzumo"){
        if(messageInfo == "1"){
            stopZumo = 1;    
        }
        if(messageInfo == "0"){
            stopZumo = 0;    
        }    
    }
    if(topic == "esp32/ladetil"){
        wantedCharge = messageInfo.toInt();
    }
    
    if(topic == "esp32/ladestasjon1"){
        if(messageInfo == "1"){
            nermeLadeStasjon = 1;    
        }    
        if(messageInfo == "0"){
            nermeLadeStasjon = 0;    
        }
    }
}

// Funksjon som reconnecter helt til vi er tilkoblet MQTT.
void reconnect() {
    // Loop helt til vi er tilkoblet MQTT
    while (!client.connected()) {
        if (client.connect("ESP8266Client")){
            client.subscribe("esp32/backchargefast");  // nød batteriboost hvis vi er tom for strøm midt på veien
            client.subscribe("esp32/stopzumo");        // Kommando for å stopp zumo i fra nodered
            client.subscribe("esp32/ladetil");         // hvor mye vi skal lade til
            client.subscribe("esp32/ladestasjon1");    // hvor langt bilen er i fra ladestajon (som regel et stort tall eller mindre en 15cm hvis vi er der)
        }
        else {
            delay(5000);
        }
    }
}


void setup() {
    Serial.begin(9600);
    setup_wifi();
    client.setServer(mqtt_server, 1883); //1883 er standard port for MQTT kommunikasjon
    client.setCallback(callback);
}

void retreveSerialDataAndPublish(){
    if(Serial.available() > 0){
        /*
        Seriel kom ser ut som eks "beginS12|345|45|56|67|78|"
        som representerer     fart, batteri, distanse, toppfart siste 60s, gjennomsnittsfart siste 60s, tid vi har vært over 70% av maxfart siste 60s.
        */
        String oldData = Serial.readStringUntil('S'); // Slik at vi alltid starter på starten av datapakken og data kommer i riktig rekkefølge
        speedReadingString = Serial.readStringUntil('|');
        batteryReadingString = Serial.readStringUntil('|');
        distanceReadingString = Serial.readStringUntil('|');
        maxSpeedReadingString = Serial.readStringUntil('|');
        averageSpeedString = Serial.readStringUntil('|');
        timeOverSeventyProsentSpeed = Serial.readStringUntil('|');
    }
  
    // Disse gjør om en string til en static char slik at den kan publishes
    static char speedChar[7];
    dtostrf(speedReadingString.toFloat(), 6, 2, speedChar);

    static char batteryChar[7];
    dtostrf(batteryReadingString.toFloat(), 6, 2, batteryChar);

    static char distanceChar[7];
    dtostrf(distanceReadingString.toFloat(), 6, 2, distanceChar);

    static char maxSpeedChar[7];
    dtostrf(maxSpeedReadingString.toFloat(), 6, 2, maxSpeedChar);

    static char averageSpeedChar[7];
    dtostrf(averageSpeedString.toFloat(), 6, 2, averageSpeedChar);

    static char overSeventyProsentSpeedChar[7];
    dtostrf(timeOverSeventyProsentSpeed.toFloat(), 6, 1, overSeventyProsentSpeedChar);
    
    
    client.publish("esp32/speed", speedChar);
    client.publish("esp32/battery", batteryChar);
    client.publish("esp32/odometer", distanceChar);
    client.publish("esp32/topspeed", maxSpeedChar);                
    client.publish("esp32/averagespeed", averageSpeedChar);             
    client.publish("esp32/seventypersentofmaxspeed", overSeventyProsentSpeedChar);
}

void sendSerialData(){
    String serialData = "beginS" + String(fast_charge) + '|' + String(stopZumo) + '|' + String(wantedCharge) + '|' + String(nermeLadeStasjon) + '|'; // S står for start av datapakken
    Serial.print(serialData); 
}


// Reconnecter til wifi hvis nødvendig
void loop() {
    if (!client.connected()) {
        reconnect();
    }
    if (!client.loop()) {
        client.connect("ESP8266Client");
    }
    if(millis() - mqttDataDelay > 400){
        retreveSerialDataAndPublish();
        mqttDataDelay = millis();
    }
    if(millis() - sendDataDelay > 200){ // var 500
        sendSerialData();
        sendDataDelay = millis();
    }         
}
