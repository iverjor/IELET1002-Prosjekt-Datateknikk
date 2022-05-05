#include <Wire.h>
#include <Zumo32U4.h>
#include <avr/pgmspace.h>
#define NUM_SENSORS 5

Zumo32U4Encoders encoders;         // Objekt som måler antall rotasjoner hjulaksen tar
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
PololuBuzzer buzzer;               // Høytaleren til bilen
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensor;
Zumo32U4IMU imu;                   // Trengs for at turnsensor bibloteket skal fungere
#include "TurnSensor.h"            // Dette er et biblotek hentet i fra eksempelkoden "rotationResist" som vi bruker for å måle retningen til bilen.
                                   // Det blir brukt til turnLeft, right funksjonene        

int leftSpeed;                     // kraften tilført til venstre motor (-400,  400)
int rightSpeed;                    // kraften tilført til høyre motor
int maxSpeed = 400;                // maxhastigeten som linjefølgeren bruker, kan fint justeres til ett område mellom ca 300 og 400
int linesensorValues[NUM_SENSORS]; // initialiserer linjesensorene
int position;                      // en linjefølger verdi i mellom 0 of 4000, som representerer hvor på en linje vi er,
unsigned long timeSinceLine;       // en variabel som brukes til å finne ut når vi sist var på linjen

int lastError = 0;                 // lastError endres kontinuerlig i linjefølger logikken. bør ikke endres. Er forskjellen i fra midten (2000)
float errorConstant1 = 0.25;       // Konstant 1 handler om erroren vi leser av nå (differanse i fra midten)
int errorConstant2 = 6;            // Konstant 2 handler om forskjellen i error i fra den forige
                                   // Begge konstanter kan finjusteres for litt annen linjefølger oppførsel. (fungerer ganske bra med 0.25, og 6) 


unsigned long updateAverageMillis = 0;  // millis variabel, Brukes i funsjonen som oppdater gjennomsnittsmålinger som snittfart, tid over 70% av maxfart osv.
unsigned long updateAverageInterval = 60000; // hvor ofte vi skal resette og oppdatere snittmålingene i milli sekunder

int maxSpeedCms = 45;                          // Toppfarten til zumo, (varierer med batteri)
int seventyPercentOfMax = maxSpeedCms*0.7;
float seventyPercentCountLive = 0;             // hver gang fart og distanse oppdateres sjekker vi om vi er over 70% av maxfart, og plusser på intervalet på denne varabelen
String seventyPercentCount = "0";              // blir regnet om til en prosent når vi er klar til å sende denne informasjonen til nodeRed 

// Rotasjon hjul og distanse kjørt
int leftCount = 0;
int rightCount = 0;
float totalDistanceLeft = 0;
float totalDistanceRight = 0;
float totalDistance = 0;
  
unsigned long lastSpeedMillis = 0;            // Siste gang vi avleste rotasjons enkoderene og bruker denne intervaltiden for å regne ut fart

float momentarySpeed = 0;                     // Denne lagrer farten vi har dette tidspungt, sendes til nodeRed. brukes også til å tappe batteri
float lastDistanceLoggedInSixtySeconds = 0;   // forige distanse logget bruker Endring i distanse / endring i tid for gjennomsnittsfart over et lengere interval
float registeredMaxSpeed = 0;                 // Vår registrerte toppfart ett lengere interval, sendes til nodeRed
float distanceDriven = 0;                     // total distanse kjørt, sendes til nodeRed
float averageSpeed;                           // Snittfarten over et lengere interval, sendes til nodeRed

// Batteri variabler
const int fullBattery = 3000;  // vårt fulle batterikapasitet
float remainingBattery = 3000; // vår faktiske batterikapasitet som blir tappet som en funksjon av fart
float lastDistanceLogged = 0;       // sammenlignes med totaldistanse over et kort interval og brukes til utregning av momentanfart

unsigned long sensorDataReadingDelay = 0;   // millis varablel som brukes til å sette et interval på hvor ofte vi skal printe info i fra zumo til esp32
unsigned long sensorDataReadingDelayInterval = 750; // hvor ofte denne printingen kan skje

unsigned long serialReadDelay = 0;            // millis varablel som brukes til å sette et interval på hvor ofte vi skal lese serial info i fra esp32 til zumo
unsigned long serialReadDelayInterval = 250;  // hvor ofte denne lesingen kan skje
 
/*
Under er info hentet i fra esp32 som skal sende forskjellige kommandoer til zumo.
Serial strengen har for som "beginS1|0|1|0|" Vi starter hver gang med å si oldMessages = Serial1.readStringUntil('S'); 
Vi sikrer da at dataen kommer i riktig rekkefølge. oldMessages gjør ingenting i koden etter dette
*/

String oldMessages = "gammel info";           
String fast_charge;
String stopZumo;
String wantedCharge;
String nermeLadeStasjon;


void calibrateLineSensors(){
    // Kalibreringsprosses for linjeSensorene
    // Bilen plasseres på tapen og snurrer mens den kalibrerer sine linjefølgersensorer
    delay(500);
    
    motors.setSpeeds(200, -200);
    for(int i = 0; i < 200; i++){
        lineSensor.calibrate();
    }
    
    motors.setSpeeds(-200, 200);
    for(int i = 0; i < 200; i++){
        lineSensor.calibrate();
    }
    
    motors.setSpeeds(0, 0);    
}


void setup(){
    /*
    Plasser zumo på en linje, trykk c. vent til den stopper å snurre. la bilen få ligge i ro mens
    gul led er på (gyro calibrering). trykk A, du kan nå plassere bilen på en linje og trykk A igjenn for start
    */
    
    Serial.begin(9600);
    Serial1.begin(9600);
    buzzer.playFrequency(1000, 100, 100);   // Spiller av en lyd slik at vi hører når ett nytt program er ferdig opplastet til bilen / bilen skrus på
    
    // Delen under omfatter kalibrering av linjefølgersensorer
    lineSensor.initFiveSensors();
    buttonC.waitForPress();
    calibrateLineSensors();

    // Kalibrer gyroskopet som brukes for rotasjon
    delay(1000);
    turnSensorSetup();
    
    // Delen under venter på et trykk for å starte void loop programmet
    buttonA.waitForPress();
    delay(500);
}    

void turnLeft(int degree){
    /*
    Funksjonen bruker biblioteket TurnSensor.h
    vi nullstiller rotasjonsdata. Bilen snur helt til ønsker vinkel er nådd
    */
    turnSensorReset();
    int wantedAngle = degree;
    int currentAngle = 0;
    motors.setSpeeds(-150, 150);
    while(wantedAngle != currentAngle){
        turnSensorUpdate();
        currentAngle = (((int32_t)turnAngle >> 16) * 360) >> 16; 
        // hvordan currentAngle fungerer forstår jeg ikke (hentet i fra "TurnSensor.h") Men den gjør om rotasjonsdata til en vikel i mellom -180, 180.
        
        // currentAngle går i fra 0 til 180 helt til den roterer over 180 grader
        // den blir da -180 til 0. Derfor sjekker jeg om den er negativ og mapper den til å fortsette sirkelen 
        if(currentAngle < 0){
            currentAngle = map(currentAngle, -180, 0, 180, 360);
        }          
    }
    motors.setSpeeds(0, 0);  
}


void turnRight(int degree){
    /*
    Funksjonen bruker biblioteket TurnSensor.h
    vi nullstiller rotasjonsdata. Bilen snur helt til ønsker vinkel er nådd
    */
    turnSensorReset();
    int wantedAngle = -degree;
    int currentAngle = 0;
    motors.setSpeeds(150, -150);
    while(wantedAngle != currentAngle){
        turnSensorUpdate();
        currentAngle = (((int32_t)turnAngle >> 16) * 360) >> 16; 
        // hvordan currentAngle fungerer forstår jeg ikke (hentet i fra "TurnSensor.h") Men den gjør om rotasjonsdata til en vikel i mellom -180, 180.
        
        // currentAngle går i fra 0 til 180 helt til den roterer over 180 grader
        // den blir da -180 til 0. Derfor sjekker jeg om den er negativ og mapper den til å fortsette sirkelen 
        if(currentAngle > 0){
            currentAngle = map(currentAngle, 180, 0, -180, -360);
        }          
    }
    // ferdig med rotasjon, kan da stoppe og gå ut a funksjonen
    motors.setSpeeds(0, 0);  
}


void driveTilJunction(){
    /*
      veien vi kommer i fra 
    |
    |
    |_____  veien vi ønsker å kjøre
    |
    |
    |
    |
     tuppen vi kjører ut av
    
    Funksjonen er laget for å takle blindvei med kryss. grunnen til av vi ikke kjører inn til "veien vi ønsker å kjøre" er at 90 graders sving logikken i vanlig void loop linjefølgeren
    sjekker om vi får en høy verdi (sensorene ser svingen) etterfulgt av en 0 eller 4000 verdi etterpå (sensorene ser gulvet), 
    vi ser en sving også fortsettelse av tape, vi snur ikke da på førte forbipassering.
    
    Hvis vi kjører av tapen i 0.5 sekunder, tar bilen en 180 graders rotasjon og aktiverer denne funksjonen.
    Den starter med å kjøre i en rett linje til vi finner tilbake til tapen. Den kjører da en kort linjefølger prosess for å stabilisere oss midt på tapen.
    etter et interval på CenteringTime (0.5 S) er den sensetiv for høye eller lave verdier den vet da at det er kryss vi ønsker å ta og svinger 90 grader i den rette retningen.  
    */
    motors.setSpeeds(200,200);

    while(position == 0 || position == 4000){
        position = lineSensor.readLine(linesensorValues);
        // for å komme tilbake på linjen
    }
    
    unsigned long centeringTimeMillis = millis();
    int centeringTime = 500;   // Tid i fra den først ser en tapebit til den kan "lete etter veikryss"
    while(true){
        // Kjører en linjefølger som etter "centeringTime" antall mS venter på ett veikryss
        // for å så svinge mot krysset og hoppe ut av loopen
        position = lineSensor.readLine(linesensorValues);
        int error = position - 2000;
        int speedDifferance = error * errorConstant1 + (error - lastError) * errorConstant2;
        lastError = error;    
        leftSpeed = maxSpeed + speedDifferance;
        rightSpeed = maxSpeed - speedDifferance;
        
        leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
        rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);
        
        if(millis() - sensorDataReadingDelay > sensorDataReadingDelayInterval){
            // oppdaterer battery, fart, distanse... sender også til esp32 som sender til nodeRed
            collectDataAndSerialPrint();
            sensorDataReadingDelay = millis();
            
            if (remainingBattery <= 0) { //Stopper motoren hvis batteriet er tomt
                leftSpeed = 0;
                rightSpeed = 0;
                remainingBattery = 0;
            }
        }
        motors.setSpeeds(leftSpeed, rightSpeed);
        
        // Kjører vanlig linjefølg i "centeringTime" ms for å sentrere oss på tapen.
        // Så etter 500ms venter vi på sving. Hvis i et kryss, vent litt slik at vi treffer tapen skikkelig.
        // Sving 90grader også gå til void loop linjefølg
        if((unsigned long)millis() - centeringTimeMillis > centeringTime){
            if(position > 2300){
                // Slik at vi treffer krysset
                motors.setSpeeds(200,200);
                delay(150);
                
                turnRight(90);
                
                // kjører litt frem så vi er på neste linjen
                motors.setSpeeds(200,200);
                delay(200);
                position = lineSensor.readLine(linesensorValues);
                break;
            }
            if(position < 1700){
                // Slik at vi treffer krysset
                motors.setSpeeds(200,200);
                delay(200);
                
                turnLeft(90);
                
                // kjører litt frem så vi er på neste linjen
                motors.setSpeeds(200,200);
                delay(200);
                position = lineSensor.readLine(linesensorValues);
                break;
            }
        }
    }
}



float readSpeed(){
    // Finner momentan fart
    int total = (leftCount + rightCount) / 2.0; //Leser av avstand
    float currentSpeed = (total / 78.0) / (((unsigned long)millis() - lastSpeedMillis) / 1000.0);
    lastSpeedMillis = millis();
    return currentSpeed;
}

float readTotalDistance() {
    // Holder oversikt over hvor langt vi har kjørt
    totalDistanceLeft += leftCount;
    totalDistanceRight += rightCount;
    totalDistance = (totalDistanceRight + totalDistanceLeft) / (2 * 7800.0);
    return totalDistance;
}


void speedometer() {
    /*
    Samler distanse, momentan fart, og sjekker om vi er over 70% av maxfart 
    */
    distanceDriven = readTotalDistance();
    float momentaryDistance = distanceDriven - lastDistanceLogged;
    momentarySpeed = readSpeed();

    if (momentarySpeed > registeredMaxSpeed){
      registeredMaxSpeed = momentarySpeed;
    }
    
    if (momentarySpeed > seventyPercentOfMax){
        float timeSinceLastReading = (millis() - sensorDataReadingDelay) / 1000.0;
        seventyPercentCountLive += timeSinceLastReading;
    }
    lastDistanceLogged = distanceDriven;
}


void collectDataAndSerialPrint(){
    /*
    Henter forskjellig data vi har på Zumo og sender det via serial monitor til esp32, som sender til nodeRed
    */
    leftCount = encoders.getCountsAndResetLeft(); //Brukes i readSpeed() og readDistance()
    rightCount = encoders.getCountsAndResetRight();
    
    speedometer(); // oppdaterer momentarySpeed, distanceDriven
    
    remainingBattery -= momentarySpeed ;  // tømmer batteri som en funksjon av fart
    int remainginBatterypercent = (remainingBattery / 3000) * 100;
    
    String speedString = String(momentarySpeed);
    String distanceDrivenString = String(distanceDriven);
    String remainingBatteryString = String(remainginBatterypercent);
    String maxSpeedString = String(registeredMaxSpeed);
    String averageSpeedString = String(averageSpeed);
    String overSeventyProsentSpeedTime = String(seventyPercentCount);
    
    // serialData er på eks format beginS55|23|1100|
    // starter med Serial.readStringUntil('S') slik at vi får data i riktik rekkefølge
    // i esp så tar vi Serial.readStringUntil('|')
    // Fart, batterikapasitet, distanse kjørt, (registrert maxfart, gjenommsnittsfart, tid vi har vært over 70% maxfart) siste x antall sekundene   
    String serialData = "trashS" + speedString + '|' + remainingBatteryString + '|' + distanceDrivenString + '|' + maxSpeedString+ '|' + averageSpeedString + '|' + overSeventyProsentSpeedTime + '|';
    Serial1.print(serialData);
} 


void updateAverages() { 
    // Oppdaterer gjennomsnittsmålinger som skjer over lengere interval,
    // snittfart, maxfart, over 70% av maxfart.
    
    averageSpeed = (distanceDriven - lastDistanceLoggedInSixtySeconds) / (millis() - updateAverageMillis) * 1000 * 100; // delta S / Delta T omgjort til cm/s
    registeredMaxSpeed = 0;
    seventyPercentCount = (seventyPercentCountLive * 100000) / (millis() - updateAverageMillis); // Dette gir oss hvor mye vi har vært over 70 % av maxfart i en prosent 0-100% av tiden
    seventyPercentCountLive = 0;
    lastDistanceLoggedInSixtySeconds = distanceDriven;
}

void readSerialData(){
    /*
    Leser serialData i fra esp32 som får i fra nodeRed.
    på format, beginS1|0|1|0|
    oldMessages Leser helt til den ser "S", vi vet da at vi får data etterpå i riktig rekkefølge
    */
    oldMessages = Serial1.readStringUntil('S'); // gammel data som ikke brukes. I tilfelle backlog
    fast_charge = Serial1.readStringUntil('|');
    stopZumo = Serial1.readStringUntil('|'); 
    wantedCharge = Serial1.readStringUntil('|'); // ønsket nivå å lade til i prosent
    nermeLadeStasjon = Serial1.readStringUntil('|');
    }

void loop(){
    position = lineSensor.readLine(linesensorValues);  // en verdi i fra 0-4000 som forteller oss hvor vi er i forhold til tapen, der 2000 er i midten
    
    // Hvis vi er på tapen. Det vil si at linjefølgeren leser en verdi mellom 1 og 3999
    if(position > 0 && position < 4000){
        timeSinceLine = millis();
    }
    
    // Hvis vi har vært av tapen i 0.5 sekund
    if((unsigned long)millis() - timeSinceLine > 500){
        // Snur bilen 180 grader og kjører tilbake til linjen også venter på et veikryss før den svinger 90 og tar krysset
        turnRight(180);
        driveTilJunction();
 
    }
    
    if((unsigned long)millis() - timeSinceLine > 3000){
        // hvis vi har ikke har sett tapen på 3 sekunder kjør donuts og prøv å lete etter banen
        while(position == 0 || position == 4000){
            motors.setSpeeds(100, 300);
            position = lineSensor.readLine(linesensorValues);
        }     
    }
    
    // pid regulering for å vite hvordan vi best kan holde oss på tapen
    int error = position - 2000;
    int speedDifferance = error * errorConstant1 + (error - lastError) * errorConstant2;
    lastError = error;
    leftSpeed = maxSpeed + speedDifferance;
    rightSpeed = maxSpeed - speedDifferance;
    leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);   // - maxSpeed hvis det blir noen bugs


    // Hvis vi er av tapen (i tilfelle av en liten glippe i veibanen) fortsett rett fremover.
    // Hvis vi er av tapen lenge nok vil "180, driveTilJunction" if betingelsen ta over
    if(position == 0 || position == 4000){
        leftSpeed = 200;
        rightSpeed = 200;
    }
    
    
    /*
    Hvis vi ser en verdi over 2600 (høyre) eller under 1400 (venstre)
    Så er vi ankommet et veikryss eller en 90 graders sving.
    hvis det er ett veikryss vil jeg forstsette rett fremmover
    hvis 90 grader må vi jo ta svingen. Ønsker å løse dette ved:
    Ser en avlesning som antyder kryss/sving, venter litt tar en ny avlesning.
    hvis den nye avlesningen er 0 eller 4000 er vi av tapen og det var ett kryss
    vi burde da svinge litt mere en 90 grader og komme oss tilbake til tapen
    eventuelt rygge litt og svinge 90 grader
    */

    
    // Sving eller kryss til høyre
    if(position > 2600 && position != 4000){
        delay(4); // denne kan økes litt for å få en mere stabil linjefølger
        position = lineSensor.readLine(linesensorValues);
        
        // Hvis dette er sant var det vi leste en 90 graders sving. Hvis det er ett kryss blir det ignorert nå
        // Den blir heller da tatt av sjekk om av tapen i 0.5 sek betingelsen litt over
        if(position == 0 || position == 4000){
            turnRight(50);
            motors.setSpeeds(200, 200);
        }
        timeSinceLine = millis();    
    }
    
    // Sving eller kryss til venstre
    if(position < 1400 && position != 0){
        delay(4); // må være tilsvarende samme som den over
        position = lineSensor.readLine(linesensorValues);
        
        // Hvis dette er sant var det vi leste en 90 graders sving 
        if(position == 0 || position == 4000){
            turnLeft(50);
            motors.setSpeeds(200, 200);
        }
        timeSinceLine = millis();  
    }
    
    while(remainingBattery <= 0) {
        /*
        Så lenge batteriet er tomt:
        Stopp motorene, avles data i fra esp32 som får data i fra nodeRed
        I nodeRed kan man trykke på Nødlading,
        "nødlader" bilen ved å rygge.
        Vi fortsetter da linjefølgeren på vanlig vis med 5 eller 10% strøm
        nok til å komme oss frem til en ladestasjon
        */
        
        leftSpeed = 0;
        rightSpeed = 0;
        remainingBattery = 0;
        motors.setSpeeds(leftSpeed, rightSpeed);
        if(Serial1.available() > 0){       
            readSerialData(); 
            serialReadDelay = millis();
        }
        if(fast_charge == "1"){             // gir oss en ladningsboost ved å rygge
            motors.setSpeeds(-200, -200);
            delay(1000);
            motors.setSpeeds(0 , 0);
            delay(500);
            remainingBattery = fullBattery * 0.20; // gir den 20% batteri   
            motors.setSpeeds(200, 200);
            delay(1000);
            fast_charge = "0"; 
        }
    }
    
    motors.setSpeeds(leftSpeed, rightSpeed);   // Sender venstre og høyrefarten som reguleringen har bestemt er best
    
    if(millis() - sensorDataReadingDelay > sensorDataReadingDelayInterval){
        // Periodisk avlesning av sensorer på bilen, dette blir brukt til blandt annet
        // Distanse kjørt, fart... Blir så sendt til esp32 -> nodeRed
        collectDataAndSerialPrint();
        sensorDataReadingDelay = millis();
    }
    
    if(millis() - serialReadDelay > serialReadDelayInterval){
        // Periodisk avlesning av info i fra nodeRed for å se om vi skal gjøre noe nytt
        if(Serial1.available() > 0){        
            readSerialData();
            serialReadDelay = millis();
        }
    }
    
    if ((millis()) - updateAverageMillis >= updateAverageInterval) {
        // funkson som kjøres på et fast interval (60 s atm)
        // oppdaterer gjennomsnittsfart, prosentvis av tiden vi har kjørt over 70% av maxfart og resetter registrert maxfart til 0
        updateAverages();
        updateAverageMillis = millis();
    }
    
    
    while(stopZumo == "1"){
        // Hvis nodeRed sier stop, så stopper vi. oppdaterer også info hentet i fra nodered får å se når vi skal starte igjenn / begynne lading hvis vi stoppet på en ladestasjon (ikke inplementert enda) 
        motors.setSpeeds(0, 0);
        
        //avlesning av info i fra nodeRed for å se om vi skal gjøre noe nytt
        if(Serial1.available() > 0){        
            readSerialData();
        }
        // Periodisk avlesning av sensorer på bilen, dette blir brukt til blandt annet
        // Distanse kjørt, fart... Blir så sendt til esp32 -> nodeRed
        collectDataAndSerialPrint();
        delay(100);
    }    
    
    if(nermeLadeStasjon == "1"){
        // Hvis vi har under 20% batteri og vi passerer ladestasjonen, så stopper vi automatisk og ladet til ønsket ladenivå (satt via nodeRed)
        if(remainingBattery < fullBattery * 0.2){
            while(1){
                motors.setSpeeds(0, 0);
                remainingBattery += 1;
                delay(10);
                
                if(remainingBattery >= fullBattery){
                    // Slik at vi aldri med uhell kommer over fullt batteri
                    remainingBattery = fullBattery;   
                }
                        
                if(millis() - sensorDataReadingDelay > sensorDataReadingDelayInterval){
                    // Periodisk avlesning av sensorer på bilen, dette blir brukt til blandt annet
                    // Distanse kjørt, fart... Blir så sendt til esp32 -> nodeRed
                    collectDataAndSerialPrint();
                    sensorDataReadingDelay = millis();
                }
                
                if(millis() - serialReadDelay > serialReadDelayInterval){ //intervalet er 500
                    // Periodisk avlesning av info i fra nodeRed for å se om vi skal gjøre noe nytt
                    if(Serial1.available() > 0){        
                        readSerialData();
                        serialReadDelay = millis();
                    }
                }
                if(remainingBattery * 100.0 / fullBattery >= wantedCharge.toInt()){
                    // når vi har ladet til ønsket nivå så stopper vi ladingen og kjører igjenn
                    break;    
                }
            }
        }   
    }
}
